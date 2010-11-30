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

#include "laser_geometry/laser_scan_geometry.h"

void
  laser_scan_geometry::projectLaser (const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud2 &cloud_out,
                                     Eigen3::ArrayXXd &co_sine_map)
{
  bool store_intensity = true;      // By default, we assume we have intensity data

  size_t n_pts = scan_in.ranges.size ();

  Eigen3::ArrayXXd ranges (n_pts, 2), output (n_pts, 2);

  // Get the ranges into Eigen format
  for (size_t i = 0; i < n_pts; ++i)
  {
    ranges (i, 0) = (double) scan_in.ranges[i];
    ranges (i, 1) = (double) scan_in.ranges[i];
  }

  // Check if we have a precomputed co_sine_map
  if (co_sine_map.rows () != (int)n_pts)
  {
    ROS_DEBUG ("[projectLaser] No precomputed map given. Computing one.");
    co_sine_map = Eigen3::ArrayXXd (n_pts, 2);
    // Spherical->Cartesian projection
    for (size_t i = 0; i < n_pts; ++i)
    {
      co_sine_map (i, 0) = cos (scan_in.angle_min + (double) i * scan_in.angle_increment);
      co_sine_map (i, 1) = sin (scan_in.angle_min + (double) i * scan_in.angle_increment);
    }
  }

  output = ranges * co_sine_map;

  // Set the output cloud accordingly
  cloud_out.header = scan_in.header;
  cloud_out.height = 1;
  cloud_out.width  = scan_in.ranges.size ();
  cloud_out.fields.resize (9);
  cloud_out.fields[0].name = "x";
  cloud_out.fields[1].name = "y";
  cloud_out.fields[2].name = "z";
  if (scan_in.intensities.size () == scan_in.ranges.size ())
  {
    cloud_out.fields[3].name = "intensity";
    cloud_out.fields[4].name = "distance";
    cloud_out.fields[5].name = "timestamp";
    cloud_out.fields[6].name = "vp_x";
    cloud_out.fields[7].name = "vp_y";
    cloud_out.fields[8].name = "vp_z";
  }
  else
  {
    cloud_out.fields.resize (8);
    cloud_out.fields[3].name = "distance";
    cloud_out.fields[4].name = "timestamp";
    cloud_out.fields[5].name = "vp_x";
    cloud_out.fields[6].name = "vp_y";
    cloud_out.fields[7].name = "vp_z";
    store_intensity = false;
  }

  // Set all the fields types accordingly
  int offset = 0;
  for (size_t s = 0; s < cloud_out.fields.size (); ++s, offset += 4)
  {
    cloud_out.fields[s].offset   = offset;
    cloud_out.fields[s].count    = 1;
    cloud_out.fields[s].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud_out.point_step = offset;
  cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
  cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
  cloud_out.is_dense = true;

  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  for (size_t i = 0; i < n_pts; ++i)
  {
    int d = 0;
    float *pstep = (float*)&cloud_out.data[i * cloud_out.point_step];

    // Copy XYZ
    pstep[d++] = output (i, 0);
    pstep[d++] = output (i, 1);
    pstep[d++] = 0;

    // Copy intensity
    if (store_intensity)
      pstep[d++] = scan_in.intensities[i];

    // Copy distance
    pstep[d++] = scan_in.ranges[i];

    // Copy timestamp
    pstep[d++] = i * scan_in.time_increment;

    // Copy viewpoint (0, 0, 0)
    pstep[d++] = 0;
    pstep[d++] = 0;
    pstep[d++] = 0;

    // Invalid measurement?
    if (scan_in.ranges[i] >= scan_in.range_max || scan_in.ranges[i] <= scan_in.range_min)
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
  }
}

void
  laser_scan_geometry::transformLaserScanToPointCloud (const std::string &target_frame, const sensor_msgs::LaserScan &scan_in,
                                                       tf::Transformer &tf, sensor_msgs::PointCloud2 &cloud_out, Eigen3::ArrayXXd &co_sine_map)
{
  bool store_intensity = true;      // By default, we assume we have intensity data
  if (scan_in.intensities.size () != scan_in.ranges.size ())
    store_intensity = false;

  projectLaser (scan_in, cloud_out, co_sine_map);

  cloud_out.header.frame_id = target_frame;

  // Extract transforms for the beginning and end of the laser scan
  ros::Time start_time = scan_in.header.stamp;
  ros::Time end_time   = scan_in.header.stamp + ros::Duration ().fromSec (scan_in.ranges.size () * scan_in.time_increment);

  tf::StampedTransform start_transform, end_transform, cur_transform ;

  tf.lookupTransform (target_frame, scan_in.header.frame_id, start_time, start_transform);
  tf.lookupTransform (target_frame, scan_in.header.frame_id, end_time, end_transform);

  double ranges_norm = 1 / ((double) scan_in.ranges.size () - 1.0);

  int distance_idx = 4;
  int vp_idx_x = 6, vp_idx_y = 7, vp_idx_z = 8;
  // Special case when intensity data doesn't exist
  if (!store_intensity)
  {
    distance_idx -= 1;
    vp_idx_x -= 1; vp_idx_y -= 1; vp_idx_z -= 1;
  }

  for (size_t i = 0; i < scan_in.ranges.size (); ++i)
  {
    // Apply the transform to the current point
    float *pstep = (float*)&cloud_out.data[i * cloud_out.point_step + 0];

    // If the input point is NaN, perform no transformation
    bool max_range_point = false;
    if (!std::isfinite (pstep[0]) || !std::isfinite (pstep[1]) || !std::isfinite (pstep[2]))
    {
      // Check for special case max range, where x==NAN, but distance=originalX
      if (std::isfinite (pstep[distance_idx]))
      {
        // copy the x-value back from range
        pstep[0] = pstep[distance_idx];
        max_range_point = true;
      }
      else
        continue;
    }

    // Assume constant motion during the laser-scan, and use slerp to compute intermediate transforms
    btScalar ratio = i * ranges_norm;

    //! \todo Make a function that performs both the slerp and linear interpolation needed to interpolate a Full Transform (Quaternion + Vector)
    // Interpolate translation
    btVector3 v (0, 0, 0);
    v.setInterpolate3 (start_transform.getOrigin (), end_transform.getOrigin (), ratio);
    cur_transform.setOrigin (v);

    // Interpolate rotation
    btQuaternion q1, q2;
    start_transform.getBasis ().getRotation (q1);
    end_transform.getBasis ().getRotation (q2);

    // Compute the slerp-ed rotation
    cur_transform.setRotation (slerp (q1, q2 , ratio));

    btVector3 point_in (pstep[0], pstep[1], pstep[2]);
    btVector3 point_out = cur_transform * point_in;

    // Copy transformed point into cloud
    pstep[0] = point_out.x ();
    pstep[1] = point_out.y ();
    pstep[2] = point_out.z ();

    // Convert the viewpoint as well
    point_in = btVector3 (pstep[vp_idx_x], pstep[vp_idx_y], pstep[vp_idx_z]);
    point_out = cur_transform * point_in;

    // Copy transformed point into cloud
    pstep[vp_idx_x] = point_out.x ();
    pstep[vp_idx_y] = point_out.y ();
    pstep[vp_idx_z] = point_out.z ();

    // Max range point? Make it invalid again, and set the transformed X to distance
    if (max_range_point)
    {
      pstep[distance_idx] = pstep[0];
      pstep[0] = std::numeric_limits<float>::quiet_NaN ();
    }
  }
}

void
  laser_scan_geometry::LaserProjection::projectLaser (const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud2 &cloud_out)
{
  boost::mutex::scoped_lock guv_lock (guv_mutex_);
  laser_scan_geometry::projectLaser (scan_in, cloud_out, co_sine_map_);
}

void
  laser_scan_geometry::LaserProjection::transformLaserScanToPointCloud (const std::string &target_frame, const sensor_msgs::LaserScan &scan_in, tf::Transformer &tf, sensor_msgs::PointCloud2 &cloud_out)
{
  laser_scan_geometry::transformLaserScanToPointCloud (target_frame, scan_in, tf, cloud_out, co_sine_map_);
}

