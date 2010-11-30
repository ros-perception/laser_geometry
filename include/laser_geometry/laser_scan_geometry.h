/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: point_types.h 33238 2010-03-11 00:46:58Z rusu $
 *
 */

#ifndef LASER_SCAN_GEOMETRY_H
#define LASER_SCAN_GEOMETRY_H

#include <Eigen3/Core>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

namespace laser_scan_geometry
{
  // NOTE: invalid scan errors (will be present in LaserScan.msg in D-Turtle)
  const float LASER_SCAN_INVALID   = -1.0;
  const float LASER_SCAN_MIN_RANGE = -2.0;
  const float LASER_SCAN_MAX_RANGE = -3.0;

  /** \brief Project a single laser scan from a linear array into a 3D point cloud.
    * The generated cloud will be in the same frame as the original laser scan.
    * \param scan_in The input laser scan
    * \param cloud_out The output point cloud
    */
  void projectLaser (const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud2 &cloud_out, Eigen3::ArrayXXd &co_sine_map);

  /** \brief Transform a sensor_msgs::LaserScan into a sensor_msgs::PointCloud in target frame
    * Transform a single laser scan from a linear array into a 3D
    * point cloud, accounting for movement of the laser over the
    * course of the scan.  In order for this transform to be
    * meaningful at a single point in time, the target_frame must
    * be a fixed reference frame.  See the tf documentation for
    * more information on fixed frames.
    *
    * \param target_frame The frame of the resulting point cloud
    * \param scan_in The input laser scan
    * \param cloud_out The output point cloud
    * \param tf a tf::Transformer object to use to perform the transform
    */
  void transformLaserScanToPointCloud (const std::string &target_frame, const sensor_msgs::LaserScan &scan_in, tf::Transformer &tf, sensor_msgs::PointCloud2 &cloud_out, Eigen3::ArrayXXd &co_sine_map);

  class LaserProjection
  {
    public:

      void projectLaser (const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud2 &cloud_out);

      void transformLaserScanToPointCloud (const std::string &target_frame, const sensor_msgs::LaserScan &scan_in, tf::Transformer &tf, sensor_msgs::PointCloud2 &cloud_out);

    private:

      Eigen3::ArrayXXd co_sine_map_;
      boost::mutex guv_mutex_;
  };
}

#endif
