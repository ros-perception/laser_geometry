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
#include <ros/assert.h>
#include <tf2/LinearMath/Transform.h>

namespace laser_geometry
{

  void
    LaserProjection::projectLaser_ (const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud & cloud_out, double range_cutoff,
                                   bool preservative, int mask)
  {
    boost::numeric::ublas::matrix<double> ranges(2, scan_in.ranges.size());

    // Fill the ranges matrix
    for (unsigned int index = 0; index < scan_in.ranges.size(); index++)
      {
        ranges(0,index) = (double) scan_in.ranges[index];
        ranges(1,index) = (double) scan_in.ranges[index];
      }

    //Do the projection
    //    NEWMAT::Matrix output = NEWMAT::SP(ranges, getUnitVectors(scan_in.angle_min, scan_in.angle_max, scan_in.angle_increment));
    boost::numeric::ublas::matrix<double> output = element_prod(ranges, getUnitVectors_(scan_in.angle_min, scan_in.angle_max, scan_in.angle_increment, scan_in.ranges.size()));

    //Stuff the output cloud
    cloud_out.header = scan_in.header;
    cloud_out.points.resize (scan_in.ranges.size());

    // Define 4 indices in the channel array for each possible value type
    int idx_intensity = -1, idx_index = -1, idx_distance = -1, idx_timestamp = -1;

    cloud_out.channels.resize(0);

    // Check if the intensity bit is set
    if ((mask & channel_option::Intensity) && scan_in.intensities.size() > 0)
    {
      int chan_size = cloud_out.channels.size();
      cloud_out.channels.resize (chan_size + 1);
      cloud_out.channels[0].name = "intensities";
      cloud_out.channels[0].values.resize (scan_in.intensities.size());
      idx_intensity = 0;
    }

    // Check if the index bit is set
    if (mask & channel_option::Index)
    {
      int chan_size = cloud_out.channels.size();
      cloud_out.channels.resize (chan_size +1);
      cloud_out.channels[chan_size].name = "index";
      cloud_out.channels[chan_size].values.resize (scan_in.ranges.size());
      idx_index = chan_size;
    }

    // Check if the distance bit is set
    if (mask & channel_option::Distance)
    {
      int chan_size = cloud_out.channels.size();
      cloud_out.channels.resize (chan_size + 1);
      cloud_out.channels[chan_size].name = "distances";
      cloud_out.channels[chan_size].values.resize (scan_in.ranges.size());
      idx_distance = chan_size;
    }

    if (mask & channel_option::Timestamp)
    {
      int chan_size = cloud_out.channels.size();
      cloud_out.channels.resize (chan_size + 1);
      cloud_out.channels[chan_size].name = "stamps";
      cloud_out.channels[chan_size].values.resize (scan_in.ranges.size());
      idx_timestamp = chan_size;
    }

    if (range_cutoff < 0)
      range_cutoff = scan_in.range_max;

    unsigned int count = 0;
    for (unsigned int index = 0; index< scan_in.ranges.size(); index++)
    {
      const float range = ranges(0, index);
      if (preservative || ((range < range_cutoff) && (range >= scan_in.range_min))) //if valid or preservative
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
          cloud_out.channels[idx_distance].values[count] = range;

        // Save intensities channel
        if (scan_in.intensities.size() >= index)
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
    cloud_out.points.resize (count);
    for (unsigned int d = 0; d < cloud_out.channels.size(); d++)
      cloud_out.channels[d].values.resize(count);
  };

const boost::numeric::ublas::matrix<double>& LaserProjection::getUnitVectors_(double angle_min, double angle_max, double angle_increment, unsigned int length)
  {
    boost::mutex::scoped_lock guv_lock(this->guv_mutex_);

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
                                                     tf::Transformer& tf, double range_cutoff, int mask)
  {
    cloud_out.header = scan_in.header;

    tf::Stamped<tf::Point> pointIn;
    tf::Stamped<tf::Point> pointOut;

    //check if the user has requested the index field
    bool requested_index = false;
    if ((mask & channel_option::Index))
      requested_index = true;

    //we need to make sure that we include the index in our mask
    //in order to guarantee that we get our timestamps right
    mask |= channel_option::Index;

    pointIn.frame_id_ = scan_in.header.frame_id;

    projectLaser_ (scan_in, cloud_out, range_cutoff, false, mask);

    cloud_out.header.frame_id = target_frame;

    // Extract transforms for the beginning and end of the laser scan
    ros::Time start_time = scan_in.header.stamp ;
    ros::Time end_time   = scan_in.header.stamp ;
    if(!scan_in.ranges.empty()) end_time += ros::Duration().fromSec( (scan_in.ranges.size()-1) * scan_in.time_increment);

    tf::StampedTransform start_transform ;
    tf::StampedTransform end_transform ;
    tf::StampedTransform cur_transform ;

    tf.lookupTransform(target_frame, scan_in.header.frame_id, start_time, start_transform) ;
    tf.lookupTransform(target_frame, scan_in.header.frame_id, end_time, end_transform) ;

    //we need to find the index of the index channel
    int index_channel_idx = -1;
    for(unsigned int i = 0; i < cloud_out.channels.size(); ++i)
    {
      if(cloud_out.channels[i].name == "index")
      {
        index_channel_idx = i;
        break;
      }
    }

    //check just in case
    ROS_ASSERT(index_channel_idx >= 0);

    for(unsigned int i = 0; i < cloud_out.points.size(); ++i)
    {
      //get the index for this point
      uint32_t pt_index = cloud_out.channels[index_channel_idx].values[i];

      // Instead, assume constant motion during the laser-scan, and use slerp to compute intermediate transforms
      tfScalar ratio = pt_index / ( (double) scan_in.ranges.size() - 1.0) ;

      //! \todo Make a function that performs both the slerp and linear interpolation needed to interpolate a Full Transform (Quaternion + Vector)

      //Interpolate translation
      tf::Vector3 v (0, 0, 0);
      v.setInterpolate3(start_transform.getOrigin(), end_transform.getOrigin(), ratio) ;
      cur_transform.setOrigin(v) ;

      //Interpolate rotation
      tf::Quaternion q1, q2 ;
      start_transform.getBasis().getRotation(q1) ;
      end_transform.getBasis().getRotation(q2) ;

      // Compute the slerp-ed rotation
      cur_transform.setRotation( slerp( q1, q2 , ratio) ) ;

      // Apply the transform to the current point
      tf::Vector3 pointIn(cloud_out.points[i].x, cloud_out.points[i].y, cloud_out.points[i].z) ;
      tf::Vector3 pointOut = cur_transform * pointIn ;

      // Copy transformed point into cloud
      cloud_out.points[i].x  = pointOut.x();
      cloud_out.points[i].y  = pointOut.y();
      cloud_out.points[i].z  = pointOut.z();
    }

    //if the user didn't request the index, we want to remove it from the channels
    if(!requested_index)
      cloud_out.channels.erase(cloud_out.channels.begin() + index_channel_idx);
  }

  void LaserProjection::projectLaser_ (const sensor_msgs::LaserScan& scan_in,
                                      sensor_msgs::PointCloud2 &cloud_out,
                                      double range_cutoff,
                                      int channel_options)
  {
    size_t n_pts = scan_in.ranges.size ();
    Eigen::ArrayXXd ranges (n_pts, 2);
    Eigen::ArrayXXd output (n_pts, 2);

    // Get the ranges into Eigen format
    for (size_t i = 0; i < n_pts; ++i)
    {
      ranges (i, 0) = (double) scan_in.ranges[i];
      ranges (i, 1) = (double) scan_in.ranges[i];
    }

    // Check if our existing co_sine_map is valid
    if (co_sine_map_.rows () != (int)n_pts || angle_min_ != scan_in.angle_min || angle_max_ != scan_in.angle_max )
    {
      ROS_DEBUG ("[projectLaser] No precomputed map given. Computing one.");
      co_sine_map_ = Eigen::ArrayXXd (n_pts, 2);
      angle_min_ = scan_in.angle_min;
      angle_max_ = scan_in.angle_max;
      // Spherical->Cartesian projection
      for (size_t i = 0; i < n_pts; ++i)
      {
        co_sine_map_ (i, 0) = cos (scan_in.angle_min + (double) i * scan_in.angle_increment);
        co_sine_map_ (i, 1) = sin (scan_in.angle_min + (double) i * scan_in.angle_increment);
      }
    }

    output = ranges * co_sine_map_;

    // Set the output cloud accordingly
    cloud_out.header = scan_in.header;
    cloud_out.height = 1;
    cloud_out.width  = scan_in.ranges.size ();
    cloud_out.fields.resize (3);
    cloud_out.fields[0].name = "x";
    cloud_out.fields[0].offset = 0;
    cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_out.fields[0].count = 1;
    cloud_out.fields[1].name = "y";
    cloud_out.fields[1].offset = 4;
    cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_out.fields[1].count = 1;
    cloud_out.fields[2].name = "z";
    cloud_out.fields[2].offset = 8;
    cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_out.fields[2].count = 1;

    // Define 4 indices in the channel array for each possible value type
    int idx_intensity = -1, idx_index = -1, idx_distance = -1, idx_timestamp = -1, idx_vpx = -1, idx_vpy = -1, idx_vpz = -1;

    //now, we need to check what fields we need to store
    int offset = 12;
    if ((channel_options & channel_option::Intensity) && scan_in.intensities.size() > 0)
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 1);
      cloud_out.fields[field_size].name = "intensity";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size].offset = offset;
      cloud_out.fields[field_size].count = 1;
      offset += 4;
      idx_intensity = field_size;
    }

    if ((channel_options & channel_option::Index))
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 1);
      cloud_out.fields[field_size].name = "index";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::INT32;
      cloud_out.fields[field_size].offset = offset;
      cloud_out.fields[field_size].count = 1;
      offset += 4;
      idx_index = field_size;
    }

    if ((channel_options & channel_option::Distance))
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 1);
      cloud_out.fields[field_size].name = "distances";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size].offset = offset;
      cloud_out.fields[field_size].count = 1;
      offset += 4;
      idx_distance = field_size;
    }

    if ((channel_options & channel_option::Timestamp))
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 1);
      cloud_out.fields[field_size].name = "stamps";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size].offset = offset;
      cloud_out.fields[field_size].count = 1;
      offset += 4;
      idx_timestamp = field_size;
    }

    if ((channel_options & channel_option::Viewpoint))
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 3);

      cloud_out.fields[field_size].name = "vp_x";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size].offset = offset;
      cloud_out.fields[field_size].count = 1;
      offset += 4;

      cloud_out.fields[field_size + 1].name = "vp_y";
      cloud_out.fields[field_size + 1].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size + 1].offset = offset;
      cloud_out.fields[field_size + 1].count = 1;
      offset += 4;

      cloud_out.fields[field_size + 2].name = "vp_z";
      cloud_out.fields[field_size + 2].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size + 2].offset = offset;
      cloud_out.fields[field_size + 2].count = 1;
      offset += 4;

      idx_vpx = field_size;
      idx_vpy = field_size + 1;
      idx_vpz = field_size + 2;
    }

    cloud_out.point_step = offset;
    cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
    cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
    cloud_out.is_dense = false;

    if (range_cutoff < 0)
      range_cutoff = scan_in.range_max;

    unsigned int count = 0;
    for (size_t i = 0; i < n_pts; ++i)
    {
      //check to see if we want to keep the point
      const float range = scan_in.ranges[i];
      if (range < range_cutoff && range >= scan_in.range_min)
      {
        float *pstep = (float*)&cloud_out.data[count * cloud_out.point_step];

        // Copy XYZ
        pstep[0] = output (i, 0);
        pstep[1] = output (i, 1);
        pstep[2] = 0;

        // Copy intensity
        if(idx_intensity != -1)
          pstep[idx_intensity] = scan_in.intensities[i];

        //Copy index
        if(idx_index != -1)
          ((int*)(pstep))[idx_index] = i;

        // Copy distance
        if(idx_distance != -1)
          pstep[idx_distance] = range;

        // Copy timestamp
        if(idx_timestamp != -1)
          pstep[idx_timestamp] = i * scan_in.time_increment;

        // Copy viewpoint (0, 0, 0)
        if(idx_vpx != -1 && idx_vpy != -1 && idx_vpz != -1)
        {
          pstep[idx_vpx] = 0;
          pstep[idx_vpy] = 0;
          pstep[idx_vpz] = 0;
        }

        //make sure to increment count
        ++count;
      }

      /* TODO: Why was this done in this way, I don't get this at all, you end up with a ton of points with NaN values
       * why can't you just leave them out?
       *
      // Invalid measurement?
      if (scan_in.ranges[i] >= range_cutoff || scan_in.ranges[i] <= scan_in.range_min)
      {
        if (scan_in.ranges[i] != LASER_SCAN_MAX_RANGE)
        {
          for (size_t s = 0; s < cloud_out.fields.size (); ++s)
            pstep[s] = bad_point;
        }
        else
        {
          // Kind of nasty thing:
          //   We keep the oringinal point information for max ranges but set x to NAN to mark the point as invalid.
          //   Since we still might need the x value we store it in the distance field
          pstep[0] = bad_point;           // X -> NAN to mark a bad point
          pstep[1] = co_sine_map (i, 1);  // Y
          pstep[2] = 0;                   // Z

          if (store_intensity)
          {
            pstep[3] = bad_point;           // Intensity -> NAN to mark a bad point
            pstep[4] = co_sine_map (i, 0);  // Distance -> Misused to store the originnal X
          }
          else
            pstep[3] = co_sine_map (i, 0);  // Distance -> Misused to store the originnal X
        }
      }
      */
    }

    //resize if necessary
    cloud_out.width = count;
    cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
    cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
  }

  void LaserProjection::transformLaserScanToPointCloud_(const std::string &target_frame,
                                                        const sensor_msgs::LaserScan &scan_in,
                                                        sensor_msgs::PointCloud2 &cloud_out,
                                                        tf2::Quaternion quat_start,
                                                        tf2::Vector3 origin_start,
                                                        tf2::Quaternion quat_end,
                                                        tf2::Vector3 origin_end,
                                                        double range_cutoff,
                                                        int channel_options)
  {
    //check if the user has requested the index field
    bool requested_index = false;
    if ((channel_options & channel_option::Index))
      requested_index = true;

    //we'll enforce that we get index values for the laser scan so that we
    //ensure that we use the correct timestamps
    channel_options |= channel_option::Index;

    projectLaser_(scan_in, cloud_out, range_cutoff, channel_options);

    //we'll assume no associated viewpoint by default
    bool has_viewpoint = false;
    uint32_t vp_x_offset = 0;

    //we need to find the offset of the intensity field in the point cloud
    //we also know that the index field is guaranteed to exist since we
    //set the channel option above. To be really safe, it might be worth
    //putting in a check at some point, but I'm just going to put in an
    //assert for now
    uint32_t index_offset = 0;
    for(unsigned int i = 0; i < cloud_out.fields.size(); ++i)
    {
      if(cloud_out.fields[i].name == "index")
      {
        index_offset = cloud_out.fields[i].offset;
      }

      //we want to check if the cloud has a viewpoint associated with it
      //checking vp_x should be sufficient since vp_x, vp_y, and vp_z all
      //get put in together
      if(cloud_out.fields[i].name == "vp_x")
      {
        has_viewpoint = true;
        vp_x_offset = cloud_out.fields[i].offset;
      }
    }

    ROS_ASSERT(index_offset > 0);

    cloud_out.header.frame_id = target_frame;

    tf2::Transform cur_transform ;

    double ranges_norm = 1 / ((double) scan_in.ranges.size () - 1.0);

    //we want to loop through all the points in the cloud
    for(size_t i = 0; i < cloud_out.width; ++i)
    {
      // Apply the transform to the current point
      float *pstep = (float*)&cloud_out.data[i * cloud_out.point_step + 0];

      //find the index of the point
      uint32_t pt_index;
      memcpy(&pt_index, &cloud_out.data[i * cloud_out.point_step + index_offset], sizeof(uint32_t));

      // Assume constant motion during the laser-scan, and use slerp to compute intermediate transforms
      tfScalar ratio = pt_index * ranges_norm;

      //! \todo Make a function that performs both the slerp and linear interpolation needed to interpolate a Full Transform (Quaternion + Vector)
      // Interpolate translation
      tf2::Vector3 v (0, 0, 0);
      v.setInterpolate3 (origin_start, origin_end, ratio);
      cur_transform.setOrigin (v);

      // Compute the slerp-ed rotation
      cur_transform.setRotation (slerp (quat_start, quat_end , ratio));

      tf2::Vector3 point_in (pstep[0], pstep[1], pstep[2]);
      tf2::Vector3 point_out = cur_transform * point_in;

      // Copy transformed point into cloud
      pstep[0] = point_out.x ();
      pstep[1] = point_out.y ();
      pstep[2] = point_out.z ();

      // Convert the viewpoint as well
      if(has_viewpoint)
      {
        float *vpstep = (float*)&cloud_out.data[i * cloud_out.point_step + vp_x_offset];
        point_in = tf2::Vector3 (vpstep[0], vpstep[1], vpstep[2]);
        point_out = cur_transform * point_in;

        // Copy transformed point into cloud
        vpstep[0] = point_out.x ();
        vpstep[1] = point_out.y ();
        vpstep[2] = point_out.z ();
      }
    }

    //if the user didn't request the index field, then we need to copy the PointCloud and drop it
    if(!requested_index)
    {
      sensor_msgs::PointCloud2 cloud_without_index;

      //copy basic meta data
      cloud_without_index.header = cloud_out.header;
      cloud_without_index.width = cloud_out.width;
      cloud_without_index.height = cloud_out.height;
      cloud_without_index.is_bigendian = cloud_out.is_bigendian;
      cloud_without_index.is_dense = cloud_out.is_dense;

      //copy the fields
      cloud_without_index.fields.resize(cloud_out.fields.size());
      unsigned int field_count = 0;
      unsigned int offset_shift = 0;
      for(unsigned int i = 0; i < cloud_out.fields.size(); ++i)
      {
        if(cloud_out.fields[i].name != "index")
        {
          cloud_without_index.fields[field_count] = cloud_out.fields[i];
          cloud_without_index.fields[field_count].offset -= offset_shift;
          ++field_count;
        }
        else
        {
          //once we hit the index, we'll set the shift
          offset_shift = 4;
        }
      }

      //resize the fields
      cloud_without_index.fields.resize(field_count);

      //compute the size of the new data
      cloud_without_index.point_step = cloud_out.point_step - offset_shift;
      cloud_without_index.row_step   = cloud_without_index.point_step * cloud_without_index.width;
      cloud_without_index.data.resize (cloud_without_index.row_step   * cloud_without_index.height);

      uint32_t i = 0;
      uint32_t j = 0;
      //copy over the data from one cloud to the other
      while (i < cloud_out.data.size())
      {
        if((i % cloud_out.point_step) < index_offset || (i % cloud_out.point_step) >= (index_offset + 4))
        {
          cloud_without_index.data[j++] = cloud_out.data[i];
        }
        i++;
      }

      //make sure to actually set the output
      cloud_out = cloud_without_index;
    }
  }

  void LaserProjection::transformLaserScanToPointCloud_ (const std::string &target_frame,
                                                         const sensor_msgs::LaserScan &scan_in,
                                                         sensor_msgs::PointCloud2 &cloud_out,
                                                         tf::Transformer &tf,
                                                         double range_cutoff,
                                                         int channel_options)
  {
    ros::Time start_time = scan_in.header.stamp;
    ros::Time end_time   = scan_in.header.stamp;
    if(!scan_in.ranges.empty()) end_time += ros::Duration ().fromSec ( (scan_in.ranges.size()-1) * scan_in.time_increment);

    tf::StampedTransform start_transform, end_transform ;

    tf.lookupTransform (target_frame, scan_in.header.frame_id, start_time, start_transform);
    tf.lookupTransform (target_frame, scan_in.header.frame_id, end_time, end_transform);

    tf::Quaternion q;
    start_transform.getBasis().getRotation(q);
    tf2::Quaternion quat_start(q.getX(), q.getY(), q.getZ(), q.getW());
    end_transform.getBasis().getRotation(q);
    tf2::Quaternion quat_end(q.getX(), q.getY(), q.getZ(), q.getW());

    tf2::Vector3 origin_start(start_transform.getOrigin().getX(),
                              start_transform.getOrigin().getY(),
                              start_transform.getOrigin().getZ());
    tf2::Vector3 origin_end(end_transform.getOrigin().getX(),
                            end_transform.getOrigin().getY(),
                            end_transform.getOrigin().getZ());
    transformLaserScanToPointCloud_(target_frame, scan_in, cloud_out,
                                    quat_start, origin_start,
                                    quat_end, origin_end,
                                    range_cutoff,
                                    channel_options);
  }

  void LaserProjection::transformLaserScanToPointCloud_ (const std::string &target_frame,
                                                         const sensor_msgs::LaserScan &scan_in,
                                                         sensor_msgs::PointCloud2 &cloud_out,
                                                         tf2::BufferCore &tf,
                                                         double range_cutoff,
                                                         int channel_options)
  {
    ros::Time start_time = scan_in.header.stamp;
    ros::Time end_time   = scan_in.header.stamp;
    if(!scan_in.ranges.empty()) end_time += ros::Duration ().fromSec ( (scan_in.ranges.size()-1) * scan_in.time_increment);

    geometry_msgs::TransformStamped start_transform = tf.lookupTransform (target_frame, scan_in.header.frame_id, start_time);
    geometry_msgs::TransformStamped end_transform = tf.lookupTransform (target_frame, scan_in.header.frame_id, end_time);

    tf2::Quaternion quat_start(start_transform.transform.rotation.x,
                               start_transform.transform.rotation.y,
                               start_transform.transform.rotation.z,
                               start_transform.transform.rotation.w);
    tf2::Quaternion quat_end(end_transform.transform.rotation.x,
                             end_transform.transform.rotation.y,
                             end_transform.transform.rotation.z,
                             end_transform.transform.rotation.w);

    tf2::Vector3 origin_start(start_transform.transform.translation.x,
                              start_transform.transform.translation.y,
                              start_transform.transform.translation.z);
    tf2::Vector3 origin_end(end_transform.transform.translation.x,
                            end_transform.transform.translation.y,
                            end_transform.transform.translation.z);
    transformLaserScanToPointCloud_(target_frame, scan_in, cloud_out,
                                    quat_start, origin_start,
                                    quat_end, origin_end,
                                    range_cutoff,
                                    channel_options);
  }

} //laser_geometry
