/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "laser_geometry/laser_geometry.h"
#include <algorithm>

namespace laser_geometry
{

  void
    LaserProjection::projectLaser_ (const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud & cloud_out, double range_cutoff, 
                                   bool preservative, int mask)
  {
    boost::numeric::ublas::matrix<double> ranges(2, scan_in.get_ranges_size());

    // Fill the ranges matrix
    for (unsigned int index = 0; index < scan_in.get_ranges_size(); index++)
      {
        ranges(0,index) = (double) scan_in.ranges[index];
        ranges(1,index) = (double) scan_in.ranges[index];
      }
    

    //Do the projection
    //    NEWMAT::Matrix output = NEWMAT::SP(ranges, getUnitVectors(scan_in.angle_min, scan_in.angle_max, scan_in.angle_increment));
    boost::numeric::ublas::matrix<double> output = element_prod(ranges, getUnitVectors_(scan_in.angle_min, scan_in.angle_max, scan_in.angle_increment, scan_in.get_ranges_size()));

    //Stuff the output cloud
    cloud_out.header = scan_in.header;
    cloud_out.set_points_size (scan_in.get_ranges_size ());

    // Define 4 indices in the channel array for each possible value type
    int idx_intensity = -1, idx_index = -1, idx_distance = -1, idx_timestamp = -1;

    cloud_out.set_channels_size(0);    

    // Check if the intensity bit is set
    if ((mask & channel_option::Intensity) && scan_in.get_intensities_size () > 0)
    {
      int chan_size = cloud_out.get_channels_size ();
      cloud_out.set_channels_size (chan_size + 1);
      cloud_out.channels[0].name = "intensities";
      cloud_out.channels[0].set_values_size (scan_in.get_intensities_size ());
      idx_intensity = 0;
    }
    
    // Check if the index bit is set
    if (mask & channel_option::Index)
    {
      int chan_size = cloud_out.get_channels_size ();
      cloud_out.set_channels_size (chan_size +1);
      cloud_out.channels[chan_size].name = "index";
      cloud_out.channels[chan_size].set_values_size (scan_in.get_ranges_size ());
      idx_index = chan_size;
    }

    // Check if the distance bit is set
    if (mask & channel_option::Distance)
    {
      int chan_size = cloud_out.get_channels_size ();
      cloud_out.set_channels_size (chan_size + 1);
      cloud_out.channels[chan_size].name = "distances";
      cloud_out.channels[chan_size].set_values_size (scan_in.get_ranges_size ());
      idx_distance = chan_size;
    }

    if (mask & channel_option::Timestamp)
    {
      int chan_size = cloud_out.get_channels_size ();
      cloud_out.set_channels_size (chan_size + 1);
      cloud_out.channels[chan_size].name = "stamps";
      cloud_out.channels[chan_size].set_values_size (scan_in.get_ranges_size ());
      idx_timestamp = chan_size;
    }

    if (range_cutoff < 0)
      range_cutoff = scan_in.range_max;
    else
      range_cutoff = std::min(range_cutoff, (double)scan_in.range_max); 
    
    unsigned int count = 0;
    for (unsigned int index = 0; index< scan_in.get_ranges_size(); index++)
    {
      if (preservative || ((ranges(0,index) < range_cutoff) && (ranges(0,index) >= scan_in.range_min))) //if valid or preservative
      {
	  
        cloud_out.points[count].x = output(0,index);
        cloud_out.points[count].y = output(1,index);
        cloud_out.points[count].z = 0.0;
	  
        //double x = cloud_out.points[count].x;
        //double y = cloud_out.points[count].y;
        //if(x*x + y*y < scan_in.range_min * scan_in.range_min){
        //  ROS_INFO("(%.2f, %.2f)", cloud_out.points[count].x, cloud_out.points[count].y);
        //}

        // Save the original point index
        if (idx_index != -1)
          cloud_out.channels[idx_index].values[count] = index;

        // Save the original point distance
        if (idx_distance != -1)
          cloud_out.channels[idx_distance].values[count] = ranges (0, index);

        // Save intensities channel
        if (scan_in.get_intensities_size() >= index)
        { /// \todo optimize and catch length difference better
          if (idx_intensity != -1)
            cloud_out.channels[idx_intensity].values[count] = scan_in.intensities[index];
        }

        // Save timestamps to seperate channel if asked for
        if( idx_timestamp != -1)
                  cloud_out.channels[idx_timestamp].values[count] = (float)index*scan_in.time_increment;

        count++;
      }
    }
   

    //downsize if necessary
    cloud_out.set_points_size (count);
    for (unsigned int d = 0; d < cloud_out.get_channels_size (); d++)
      cloud_out.channels[d].set_values_size(count);
  };

const boost::numeric::ublas::matrix<double>& LaserProjection::getUnitVectors_(double angle_min, double angle_max, double angle_increment, unsigned int length)
  {
    //construct string for lookup in the map
    std::stringstream anglestring;
    anglestring <<angle_min<<","<<angle_max<<","<<angle_increment<<","<<length;
    std::map<std::string, boost::numeric::ublas::matrix<double>* >::iterator it;
    it = unit_vector_map_.find(anglestring.str());
    //check the map for presense
    if (it != unit_vector_map_.end())
      return *((*it).second);     //if present return

    boost::numeric::ublas::matrix<double> * tempPtr = new boost::numeric::ublas::matrix<double>(2,length);
    for (unsigned int index = 0;index < length; index++)
      {
        (*tempPtr)(0,index) = cos(angle_min + (double) index * angle_increment);
        (*tempPtr)(1,index) = sin(angle_min + (double) index * angle_increment);
      }
    //store 
    unit_vector_map_[anglestring.str()] = tempPtr;
    //and return
    return *tempPtr;
  };


  LaserProjection::~LaserProjection()
  {
    std::map<std::string, boost::numeric::ublas::matrix<double>*>::iterator it;
    it = unit_vector_map_.begin();
    while (it != unit_vector_map_.end())
      {
        delete (*it).second;
        it++;
      }
  };

  void
    LaserProjection::transformLaserScanToPointCloud_ (const std::string &target_frame, sensor_msgs::PointCloud &cloud_out, const sensor_msgs::LaserScan &scan_in,
                                                     tf::Transformer& tf, int mask, bool preservative)
  {
    cloud_out.header = scan_in.header;
    cloud_out.header.frame_id = target_frame;
    cloud_out.set_points_size (scan_in.get_ranges_size());
    
    // Define 4 indices in the channel array for each possible value type
    int idx_intensity = -1, idx_index = -1, idx_distance = -1, idx_timestamp = -1;


    cloud_out.set_channels_size(0);

    // Check if the intensity bit is set
    if ((mask & channel_option::Intensity) && scan_in.get_intensities_size () > 0)
    {
      int chan_size = cloud_out.get_channels_size ();
      cloud_out.set_channels_size (chan_size + 1);
      cloud_out.channels[0].name = "intensities";
      cloud_out.channels[0].set_values_size (scan_in.get_intensities_size ());
      idx_intensity = 0;
    }
    
    // Check if the index bit is set
    if (mask & channel_option::Index)
    {
      int chan_size = cloud_out.get_channels_size ();
      cloud_out.set_channels_size (chan_size + 1);
      cloud_out.channels[chan_size].name = "index";
      cloud_out.channels[chan_size].set_values_size (scan_in.get_ranges_size ());
      idx_index = chan_size;
    }

    // Check if the distance bit is set
    if (mask & channel_option::Distance)
    {
      int chan_size = cloud_out.get_channels_size ();
      cloud_out.set_channels_size (chan_size + 1);
      cloud_out.channels[chan_size].name = "distances";
      cloud_out.channels[chan_size].set_values_size (scan_in.get_ranges_size ());
      idx_distance = chan_size;
    }

    if (mask & channel_option::Timestamp)
    {
      int chan_size = cloud_out.get_channels_size ();
      cloud_out.set_channels_size (chan_size + 1);
      cloud_out.channels[chan_size].name = "stamps";
      cloud_out.channels[chan_size].set_values_size (scan_in.get_ranges_size ());
      idx_timestamp = chan_size;
    }

    tf::Stamped<tf::Point> pointIn;
    tf::Stamped<tf::Point> pointOut;

    pointIn.frame_id_ = scan_in.header.frame_id;

    ///\todo this can be optimized
    sensor_msgs::PointCloud intermediate; //optimize out

    projectLaser_ (scan_in, intermediate, -1.0, true, mask);

    // Extract transforms for the beginning and end of the laser scan
    ros::Time start_time = scan_in.header.stamp ;
    ros::Time end_time   = scan_in.header.stamp + ros::Duration().fromSec(scan_in.get_ranges_size()*scan_in.time_increment) ;

    tf::StampedTransform start_transform ;
    tf::StampedTransform end_transform ;
    tf::StampedTransform cur_transform ;

    tf.lookupTransform(target_frame, scan_in.header.frame_id, start_time, start_transform) ;
    tf.lookupTransform(target_frame, scan_in.header.frame_id, end_time, end_transform) ;

    unsigned int count = 0;  
    for (unsigned int i = 0; i < scan_in.get_ranges_size(); i++)
    {
      if (preservative || (scan_in.ranges[i] < scan_in.range_max && scan_in.ranges[i] > scan_in.range_min)) //only when valid
      {
        // Looking up transforms in tree is too expensive. Need more optimized way
        /*
           pointIn = tf::Stamped<tf::Point>(btVector3(intermediate.points[i].x, intermediate.points[i].y, intermediate.points[i].z), 
           ros::Time(scan_in.header.stamp.to_ull() + (uint64_t) (scan_in.time_increment * 1000000000)),
           pointIn.frame_id_ = scan_in.header.frame_id);///\todo optimize to no copy
           transformPoint(target_frame, pointIn, pointOut);
           */

        // Instead, assume constant motion during the laser-scan, and use slerp to compute intermediate transforms
        btScalar ratio = i / ( (double) scan_in.get_ranges_size() - 1.0) ;

        //! \todo Make a function that performs both the slerp and linear interpolation needed to interpolate a Full Transform (Quaternion + Vector)

        //Interpolate translation
        btVector3 v (0, 0, 0);
        v.setInterpolate3(start_transform.getOrigin(), end_transform.getOrigin(), ratio) ;
        cur_transform.setOrigin(v) ;

        //Interpolate rotation
        btQuaternion q1, q2 ;
        start_transform.getBasis().getRotation(q1) ;
        end_transform.getBasis().getRotation(q2) ;

        // Compute the slerp-ed rotation
        cur_transform.setRotation( slerp( q1, q2 , ratio) ) ;

        // Apply the transform to the current point
        btVector3 pointIn(intermediate.points[i].x, intermediate.points[i].y, intermediate.points[i].z) ;
        btVector3 pointOut = cur_transform * pointIn ;

        // Copy transformed point into cloud
        cloud_out.points[count].x  = pointOut.x();
        cloud_out.points[count].y  = pointOut.y();
        cloud_out.points[count].z  = pointOut.z();

        // Copy index over from projected point cloud
        if (idx_index != -1)
          cloud_out.channels[idx_index].values[count] = intermediate.channels[idx_index].values[i];
        
        // Save the original point distance
        if (idx_distance != -1)
          cloud_out.channels[idx_distance].values[count] = scan_in.ranges[i];

        if (scan_in.get_intensities_size() >= i)
        { /// \todo optimize and catch length difference better
          if (idx_intensity != -1)
            cloud_out.channels[idx_intensity].values[count] = scan_in.intensities[i];
        }

        if (idx_timestamp != -1)
          cloud_out.channels[idx_timestamp].values[count] = (float)i * scan_in.time_increment;

        count++;
      }

    }
    //downsize if necessary
    cloud_out.set_points_size (count);
    for (unsigned int d = 0; d < cloud_out.get_channels_size (); d++)
      cloud_out.channels[d].set_values_size (count);
  }


} //laser_geometry
