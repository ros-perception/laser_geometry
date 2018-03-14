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

#include <gtest/gtest.h>
#include <sys/time.h>
#include <math.h>

#include "rclcpp/rclcpp.hpp"

#include "laser_geometry/laser_geometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#define PROJECTION_TEST_RANGE_MIN (0.23)
#define PROJECTION_TEST_RANGE_MAX (40.0) 


class BuildScanException { };

struct ScanOptions {
  double range_;
  double intensity_;
  double ang_min_;
  double ang_max_;
  double ang_increment_;
  rclcpp::Duration scan_time_;

  ScanOptions(double range, double intensity,
    double ang_min, double ang_max, double ang_increment,
    rclcpp::Duration scan_time) :
    range_(range),
    intensity_(intensity),
    ang_min_(ang_min),
    ang_max_(ang_max),
    ang_increment_(ang_increment),
    scan_time_(scan_time) {}
};

sensor_msgs::msg::LaserScan build_constant_scan(const ScanOptions & options)
{
  if (((options.ang_max_ - options.ang_min_) / options.ang_increment_) < 0)
    throw (BuildScanException());

  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = rclcpp::Clock().now();
  scan.header.frame_id = "laser_frame";
  scan.angle_min = options.ang_min_;
  scan.angle_max = options.ang_max_;
  scan.angle_increment = options.ang_increment_;
  scan.scan_time = options.scan_time_.nanoseconds();
  scan.range_min = PROJECTION_TEST_RANGE_MIN;
  scan.range_max = PROJECTION_TEST_RANGE_MAX;
  uint32_t i = 0;
  for(; options.ang_min_ + i * options.ang_increment_ < options.ang_max_; i++)
  {
    scan.ranges.push_back(options.range_);
    scan.intensities.push_back(options.intensity_);
  }

  scan.time_increment = options.scan_time_.nanoseconds()/(double)i;

  return scan;
};

TEST(laser_geometry, projectLaser2)
{
  double tolerance = 1e-12;
  laser_geometry::LaserProjection projector;  

  std::vector<double> ranges, intensities, min_angles, max_angles, angle_increments;
  std::vector<rclcpp::Duration> increment_times, scan_times;

  ranges.push_back(-1.0);
  ranges.push_back(1.0);
  ranges.push_back(2.0);
  ranges.push_back(3.0);
  ranges.push_back(4.0);
  ranges.push_back(5.0);
  ranges.push_back(100.0);

  intensities.push_back(1.0);
  intensities.push_back(2.0);
  intensities.push_back(3.0);
  intensities.push_back(4.0);
  intensities.push_back(5.0);

  min_angles.push_back(-M_PI);
  min_angles.push_back(-M_PI/1.5);
  min_angles.push_back(-M_PI/2);
  min_angles.push_back(-M_PI/4);
  min_angles.push_back(-M_PI/8);

  max_angles.push_back(M_PI);
  max_angles.push_back(M_PI/1.5);
  max_angles.push_back(M_PI/2);
  max_angles.push_back(M_PI/4);
  max_angles.push_back(M_PI/8);

  //  angle_increments.push_back(-M_PI/180); // -one degree
  angle_increments.push_back(M_PI/180); // one degree
  angle_increments.push_back(M_PI/360); // half degree
  angle_increments.push_back(M_PI/720); // quarter degree

  scan_times.push_back(rclcpp::Duration(1/40));
  scan_times.push_back(rclcpp::Duration(1/20));

  std::vector<ScanOptions> options;
  for(auto range : ranges) {
    for(auto intensity : intensities) {
      for(auto min_angle : min_angles) {
        for(auto max_angle : max_angles) {
          for(auto angle_increment : angle_increments) {
            for(auto scan_time : scan_times) {
              options.push_back(ScanOptions(
                range, intensity, min_angle, max_angle, angle_increment, scan_time));
            }
          }
        }
      }
    }
  }

  for (auto option : options){
    try
    {
      //        printf("%f %f %f %f %f %f\n", range, intensity, min_angle, max_angle, angle_increment, scan_time.toSec());
  sensor_msgs::msg::LaserScan scan = build_constant_scan(option);

  sensor_msgs::msg::PointCloud2 cloud_out;
  projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Index);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)4);
  projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Intensity);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)4);

  projector.projectLaser(scan, cloud_out, -1.0);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)5);
  projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)5);

  projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)6);

  projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance | laser_geometry::channel_option::Timestamp);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)7);

  unsigned int valid_points = 0;
  for (unsigned int i = 0; i < scan.ranges.size(); i++)
    if (scan.ranges[i] <= PROJECTION_TEST_RANGE_MAX && scan.ranges[i] >= PROJECTION_TEST_RANGE_MIN)
      valid_points ++;

  EXPECT_EQ(valid_points, cloud_out.width);

  uint32_t x_offset = 0;
  uint32_t y_offset = 0;
  uint32_t z_offset = 0;
  uint32_t intensity_offset = 0;
  uint32_t index_offset = 0;
  uint32_t distance_offset = 0;
  uint32_t stamps_offset = 0;
  for (std::vector<sensor_msgs::msg::PointField>::iterator f = cloud_out.fields.begin(); f != cloud_out.fields.end(); f++) {
      if (f->name == "x") x_offset = f->offset;
      if (f->name == "y") y_offset = f->offset;
      if (f->name == "z") z_offset = f->offset;
      if (f->name == "intensity") intensity_offset = f->offset;
      if (f->name == "index") index_offset = f->offset;
      if (f->name == "distances") distance_offset = f->offset;
      if (f->name == "stamps") stamps_offset = f->offset;
  }

  for (unsigned int i = 0; i < cloud_out.width; i++) {

    EXPECT_NEAR(*(float*)&cloud_out.data[i*cloud_out.point_step + x_offset] , (float)((double)(scan.ranges[i]) * cos((double)(scan.angle_min) + i * (double)(scan.angle_increment))), tolerance);
    EXPECT_NEAR(*(float*)&cloud_out.data[i*cloud_out.point_step + y_offset] , (float)((double)(scan.ranges[i]) * sin((double)(scan.angle_min) + i * (double)(scan.angle_increment))), tolerance);
    EXPECT_NEAR(*(float*)&cloud_out.data[i*cloud_out.point_step + z_offset] , 0, tolerance);
    EXPECT_NEAR(*(float*)&cloud_out.data[i*cloud_out.point_step + intensity_offset] , scan.intensities[i], tolerance);//intensity \todo determine this by lookup not hard coded order
    EXPECT_NEAR(*(uint32_t*)&cloud_out.data[i*cloud_out.point_step + index_offset], i, tolerance);//index
    EXPECT_NEAR(*(float*)&cloud_out.data[i*cloud_out.point_step + distance_offset], scan.ranges[i], tolerance);//ranges
    EXPECT_NEAR(*(float*)&cloud_out.data[i*cloud_out.point_step + stamps_offset], (float)i * scan.time_increment, tolerance);//timestamps
  };
    }
    catch (BuildScanException &ex)
    {
      // make sure it is not a false exception
      if ((option.ang_max_ - option.ang_min_) / option.ang_increment_ > 0.0) {
        FAIL();
      }
    }
  }
}

TEST(laser_geometry, transformLaserScanToPointCloud2)
{

  tf2::BufferCore tf2;

  double tolerance = 1e-12;
  laser_geometry::LaserProjection projector;  

  std::vector<double> ranges, intensities, min_angles, max_angles, angle_increments;
  std::vector<rclcpp::Duration> increment_times, scan_times;

  ranges.push_back(-1.0);
  ranges.push_back(1.0);
  ranges.push_back(2.0);
  ranges.push_back(3.0);
  ranges.push_back(4.0);
  ranges.push_back(5.0);
  ranges.push_back(100.0);

  intensities.push_back(1.0);
  intensities.push_back(2.0);
  intensities.push_back(3.0);
  intensities.push_back(4.0);
  intensities.push_back(5.0);

  min_angles.push_back(-M_PI);
  min_angles.push_back(-M_PI/1.5);
  min_angles.push_back(-M_PI/2);
  min_angles.push_back(-M_PI/4);
  min_angles.push_back(-M_PI/8);

  max_angles.push_back(M_PI);
  max_angles.push_back(M_PI/1.5);
  max_angles.push_back(M_PI/2);
  max_angles.push_back(M_PI/4);
  max_angles.push_back(M_PI/8);

  angle_increments.push_back(-M_PI/180); // -one degree
  angle_increments.push_back(M_PI/180); // one degree
  angle_increments.push_back(M_PI/360); // half degree
  angle_increments.push_back(M_PI/720); // quarter degree

  scan_times.push_back(rclcpp::Duration(1/40));
  scan_times.push_back(rclcpp::Duration(1/20));


  std::vector<ScanOptions> options;
  for(auto range : ranges) {
    for(auto intensity : intensities) {
      for(auto min_angle : min_angles) {
        for(auto max_angle : max_angles) {
          for(auto angle_increment : angle_increments) {
            for(auto scan_time : scan_times) {
              options.push_back(ScanOptions(
                range, intensity, min_angle, max_angle, angle_increment, scan_time));
            }
          }
        }
      }
    }
  }

  for (auto option : options){
    try
    {
      //        printf("%f %f %f %f %f %f\n", range, intensity, min_angle, max_angle, angle_increment, scan_time.toSec());
      sensor_msgs::msg::LaserScan scan = build_constant_scan(option);

      scan.header.frame_id = "laser_frame";

  sensor_msgs::msg::PointCloud2 cloud_out;
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2, -1.0, laser_geometry::channel_option::None);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)3);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2, -1.0, laser_geometry::channel_option::Index);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)4);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2, -1.0, laser_geometry::channel_option::Intensity);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)4);

  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)5);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)5);

  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)6);

  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance | laser_geometry::channel_option::Timestamp);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)7);

  EXPECT_EQ(cloud_out.is_dense, false);

  unsigned int valid_points = 0;
  for (unsigned int i = 0; i < scan.ranges.size(); i++)
    if (scan.ranges[i] <= PROJECTION_TEST_RANGE_MAX && scan.ranges[i] >= PROJECTION_TEST_RANGE_MIN)
      valid_points ++;
  EXPECT_EQ(valid_points, cloud_out.width);

  uint32_t x_offset = 0;
  uint32_t y_offset = 0;
  uint32_t z_offset = 0;
  uint32_t intensity_offset = 0;
  uint32_t index_offset = 0;
  uint32_t distance_offset = 0;
  uint32_t stamps_offset = 0;
  for (std::vector<sensor_msgs::msg::PointField>::iterator f = cloud_out.fields.begin(); f != cloud_out.fields.end(); f++)
  {
    if (f->name == "x") x_offset = f->offset;
    if (f->name == "y") y_offset = f->offset;
    if (f->name == "z") z_offset = f->offset;
    if (f->name == "intensity") intensity_offset = f->offset;
    if (f->name == "index") index_offset = f->offset;
    if (f->name == "distances") distance_offset = f->offset;
    if (f->name == "stamps") stamps_offset = f->offset;
  }

  for (unsigned int i = 0; i < cloud_out.width; i++)
  {
    EXPECT_NEAR(*(float*)&cloud_out.data[i*cloud_out.point_step + x_offset] , (float)((double)(scan.ranges[i]) * cos((double)(scan.angle_min) + i * (double)(scan.angle_increment))), tolerance);
    EXPECT_NEAR(*(float*)&cloud_out.data[i*cloud_out.point_step + y_offset] , (float)((double)(scan.ranges[i]) * sin((double)(scan.angle_min) + i * (double)(scan.angle_increment))), tolerance);
    EXPECT_NEAR(*(float*)&cloud_out.data[i*cloud_out.point_step + z_offset] , 0, tolerance);
    EXPECT_NEAR(*(float*)&cloud_out.data[i*cloud_out.point_step + intensity_offset] , scan.intensities[i], tolerance);//intensity \todo determine this by lookup not hard coded order
    EXPECT_NEAR(*(uint32_t*)&cloud_out.data[i*cloud_out.point_step + index_offset], i, tolerance);//index
    EXPECT_NEAR(*(float*)&cloud_out.data[i*cloud_out.point_step + distance_offset], scan.ranges[i], tolerance);//ranges
    EXPECT_NEAR(*(float*)&cloud_out.data[i*cloud_out.point_step + stamps_offset], (float)i * scan.time_increment, tolerance);//timestamps

  };
    }
    catch (BuildScanException &ex)
    {
      // make sure it is not a false exception
      if ((option.ang_max_ - option.ang_min_) / option.ang_increment_ > 0.0) {
        FAIL();
      }
    }
  }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
