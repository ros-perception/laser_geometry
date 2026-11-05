// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#define _USE_MATH_DEFINES
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/buffer_core.hpp"
#include "tf2/exceptions.hpp"

#define PROJECTION_TEST_RANGE_MIN (0.23f)
#define PROJECTION_TEST_RANGE_MAX (40.0f)

#define PI static_cast<float>(M_PI)

struct ScanOptions
{
  float range_;
  float intensity_;
  float ang_min_;
  float ang_max_;
  float ang_increment_;
  rclcpp::Duration scan_time_;

  ScanOptions(
    float range, float intensity,
    float ang_min, float ang_max, float ang_increment,
    rclcpp::Duration scan_time)
  : range_(range),
    intensity_(intensity),
    ang_min_(ang_min),
    ang_max_(ang_max),
    ang_increment_(ang_increment),
    scan_time_(scan_time) {}
};

sensor_msgs::msg::LaserScan build_constant_scan(const ScanOptions & options)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = rclcpp::Clock().now();
  scan.header.frame_id = "laser_frame";
  scan.angle_min = options.ang_min_;
  scan.angle_max = options.ang_max_;
  scan.angle_increment = options.ang_increment_;
  scan.scan_time = static_cast<float>(options.scan_time_.seconds());
  scan.range_min = PROJECTION_TEST_RANGE_MIN;
  scan.range_max = PROJECTION_TEST_RANGE_MAX;
  uint32_t i = 0;
  for (; options.ang_min_ + i * options.ang_increment_ < options.ang_max_; i++) {
    scan.ranges.push_back(options.range_);
    scan.intensities.push_back(options.intensity_);
  }

  scan.time_increment =
    static_cast<float>(options.scan_time_.seconds() / static_cast<double>(i));

  return scan;
}

template<typename T>
T cloudData(sensor_msgs::msg::PointCloud2 cloud_out, uint32_t index)
{
  return *reinterpret_cast<T *>(&cloud_out.data[index]);
}

TEST(laser_geometry, projectLaser2) {
  double tolerance = 1e-12;
  laser_geometry::LaserProjection projector;

  std::vector<float> ranges, intensities, min_angles, max_angles, angle_increments;
  std::vector<rclcpp::Duration> increment_times, scan_times;

  ranges.push_back(-1.0f);
  ranges.push_back(1.0f);
  ranges.push_back(2.0f);
  ranges.push_back(100.0f);

  intensities.push_back(1.0f);
  intensities.push_back(2.0f);
  intensities.push_back(5.0f);

  min_angles.push_back(-PI);
  min_angles.push_back(-PI / 1.5f);
  min_angles.push_back(-PI / 8);

  max_angles.push_back(PI);
  max_angles.push_back(PI / 1.5f);
  max_angles.push_back(PI / 8);

  angle_increments.push_back(-PI / 180);  // -one degree
  angle_increments.push_back(PI / 180);  // one degree
  angle_increments.push_back(PI / 720);  // quarter degree

  scan_times.push_back(rclcpp::Duration::from_seconds(1. / 40));
  scan_times.push_back(rclcpp::Duration::from_seconds(1. / 20));

  std::vector<ScanOptions> options;
  for (auto range : ranges) {
    for (auto intensity : intensities) {
      for (auto min_angle : min_angles) {
        for (auto max_angle : max_angles) {
          for (auto angle_increment : angle_increments) {
            for (auto scan_time : scan_times) {
              options.push_back(
                ScanOptions(
                  range, intensity, min_angle, max_angle, angle_increment, scan_time));
            }
          }
        }
      }
    }
  }

  for (auto option : options) {
    if ((option.ang_max_ - option.ang_min_) / option.ang_increment_ < 0) {
      continue;
    }
    sensor_msgs::msg::LaserScan scan = build_constant_scan(option);

    sensor_msgs::msg::PointCloud2 cloud_out;
    projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Index);
    EXPECT_EQ(cloud_out.fields.size(), 4u);
    projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Intensity);
    EXPECT_EQ(cloud_out.fields.size(), 4u);

    projector.projectLaser(scan, cloud_out, -1.0);
    EXPECT_EQ(cloud_out.fields.size(), 5u);
    projector.projectLaser(
      scan, cloud_out, -1.0,
      laser_geometry::channel_option::Intensity |
      laser_geometry::channel_option::Index);
    EXPECT_EQ(cloud_out.fields.size(), 5u);

    projector.projectLaser(
      scan, cloud_out, -1.0,
      laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index |
      laser_geometry::channel_option::Distance);
    EXPECT_EQ(cloud_out.fields.size(), 6u);

    projector.projectLaser(
      scan, cloud_out, -1.0,
      laser_geometry::channel_option::Intensity |
      laser_geometry::channel_option::Index |
      laser_geometry::channel_option::Distance |
      laser_geometry::channel_option::Timestamp);
    EXPECT_EQ(cloud_out.fields.size(), 7u);

    unsigned int valid_points = 0;
    for (unsigned int i = 0; i < scan.ranges.size(); i++) {
      if (scan.ranges[i] <= PROJECTION_TEST_RANGE_MAX &&
        scan.ranges[i] >= PROJECTION_TEST_RANGE_MIN)
      {
        valid_points++;
      }
    }

    EXPECT_EQ(valid_points, cloud_out.width);

    uint32_t x_offset = 0;
    uint32_t y_offset = 0;
    uint32_t z_offset = 0;
    uint32_t intensity_offset = 0;
    uint32_t index_offset = 0;
    uint32_t distance_offset = 0;
    uint32_t stamps_offset = 0;
    for (std::vector<sensor_msgs::msg::PointField>::iterator f = cloud_out.fields.begin();
      f != cloud_out.fields.end(); f++)
    {
      if (f->name == "x") {x_offset = f->offset;}
      if (f->name == "y") {y_offset = f->offset;}
      if (f->name == "z") {z_offset = f->offset;}
      if (f->name == "intensity") {intensity_offset = f->offset;}
      if (f->name == "index") {index_offset = f->offset;}
      if (f->name == "distances") {distance_offset = f->offset;}
      if (f->name == "stamps") {stamps_offset = f->offset;}
    }

    for (unsigned int i = 0; i < cloud_out.width; i++) {
      EXPECT_NEAR(
        cloudData<float>(cloud_out, i * cloud_out.point_step + x_offset),
        static_cast<float>(static_cast<double>(scan.ranges[i]) *
        cos(static_cast<double>(scan.angle_min) + i * static_cast<double>(scan.angle_increment))),
        tolerance);
      EXPECT_NEAR(
        cloudData<float>(cloud_out, i * cloud_out.point_step + y_offset),
        static_cast<float>(static_cast<double>(scan.ranges[i]) *
        sin(static_cast<double>(scan.angle_min) + i * static_cast<double>(scan.angle_increment))),
        tolerance);
      EXPECT_NEAR(cloudData<float>(cloud_out, i * cloud_out.point_step + z_offset), 0, tolerance);
      EXPECT_NEAR(
        cloudData<float>(cloud_out, i * cloud_out.point_step + intensity_offset),
        scan.intensities[i], tolerance);  // intensity
      EXPECT_NEAR(
        cloudData<uint32_t>(cloud_out, i * cloud_out.point_step + index_offset), i,
        tolerance);  // index
      EXPECT_NEAR(
        cloudData<float>(cloud_out, i * cloud_out.point_step + distance_offset),
        scan.ranges[i], tolerance);  // ranges
      EXPECT_NEAR(
        cloudData<float>(cloud_out, i * cloud_out.point_step + stamps_offset),
        (float)i * scan.time_increment, tolerance);  // timestamps
    }
  }
}

TEST(laser_geometry, transformLaserScanToPointCloud2) {
  tf2::BufferCore tf_buffer;

  // Register "laser_frame" in the buffer with a static identity transform so that
  // same-frame lookups succeed regardless of the scan timestamp. The transform
  // applied to the cloud is therefore the identity, which lets us verify the
  // projected point coordinates against the same trig formula as projectLaser2.
  geometry_msgs::msg::TransformStamped identity;
  identity.header.frame_id = "world";
  identity.child_frame_id = "laser_frame";
  identity.transform.rotation.w = 1.0;
  tf_buffer.setTransform(identity, "test_authority", true);

  double tolerance = 1e-6;
  laser_geometry::LaserProjection projector;

  std::vector<float> ranges, intensities, min_angles, max_angles, angle_increments;
  std::vector<rclcpp::Duration> scan_times;

  ranges.push_back(-1.0f);
  ranges.push_back(1.0f);
  ranges.push_back(2.0f);
  ranges.push_back(100.0f);

  intensities.push_back(1.0f);
  intensities.push_back(2.0f);
  intensities.push_back(5.0f);

  min_angles.push_back(-PI);
  min_angles.push_back(-PI / 1.5f);
  min_angles.push_back(-PI / 8);

  max_angles.push_back(PI);
  max_angles.push_back(PI / 1.5f);
  max_angles.push_back(PI / 8);

  angle_increments.push_back(-PI / 180);  // -one degree
  angle_increments.push_back(PI / 180);  // one degree
  angle_increments.push_back(PI / 720);  // quarter degree

  scan_times.push_back(rclcpp::Duration::from_seconds(1. / 40));
  scan_times.push_back(rclcpp::Duration::from_seconds(1. / 20));

  std::vector<ScanOptions> options;
  for (auto range : ranges) {
    for (auto intensity : intensities) {
      for (auto min_angle : min_angles) {
        for (auto max_angle : max_angles) {
          for (auto angle_increment : angle_increments) {
            for (auto scan_time : scan_times) {
              options.push_back(
                ScanOptions(
                  range, intensity, min_angle, max_angle, angle_increment, scan_time));
            }
          }
        }
      }
    }
  }

  for (auto option : options) {
    if ((option.ang_max_ - option.ang_min_) / option.ang_increment_ < 0) {
      continue;
    }
    sensor_msgs::msg::LaserScan scan = build_constant_scan(option);

    sensor_msgs::msg::PointCloud2 cloud_out;
    projector.transformLaserScanToPointCloud(
      scan.header.frame_id, scan, cloud_out, tf_buffer, -1.0,
      laser_geometry::channel_option::None);
    EXPECT_EQ(cloud_out.fields.size(), 3u);
    projector.transformLaserScanToPointCloud(
      scan.header.frame_id, scan, cloud_out, tf_buffer, -1.0,
      laser_geometry::channel_option::Index);
    EXPECT_EQ(cloud_out.fields.size(), 4u);
    projector.transformLaserScanToPointCloud(
      scan.header.frame_id, scan, cloud_out, tf_buffer, -1.0,
      laser_geometry::channel_option::Intensity);
    EXPECT_EQ(cloud_out.fields.size(), 4u);

    projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf_buffer);
    EXPECT_EQ(cloud_out.fields.size(), 5u);
    projector.transformLaserScanToPointCloud(
      scan.header.frame_id, scan, cloud_out, tf_buffer, -1.0,
      laser_geometry::channel_option::Intensity |
      laser_geometry::channel_option::Index);
    EXPECT_EQ(cloud_out.fields.size(), 5u);

    projector.transformLaserScanToPointCloud(
      scan.header.frame_id, scan, cloud_out, tf_buffer, -1.0,
      laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index |
      laser_geometry::channel_option::Distance);
    EXPECT_EQ(cloud_out.fields.size(), 6u);

    projector.transformLaserScanToPointCloud(
      scan.header.frame_id, scan, cloud_out, tf_buffer, -1.0,
      laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index |
      laser_geometry::channel_option::Distance |
      laser_geometry::channel_option::Timestamp);
    EXPECT_EQ(cloud_out.fields.size(), 7u);

    EXPECT_EQ(cloud_out.is_dense, false);

    unsigned int valid_points = 0;
    for (unsigned int i = 0; i < scan.ranges.size(); i++) {
      if (scan.ranges[i] <= PROJECTION_TEST_RANGE_MAX &&
        scan.ranges[i] >= PROJECTION_TEST_RANGE_MIN)
      {
        valid_points++;
      }
    }
    EXPECT_EQ(valid_points, cloud_out.width);

    uint32_t x_offset = 0;
    uint32_t y_offset = 0;
    uint32_t z_offset = 0;
    uint32_t intensity_offset = 0;
    uint32_t distance_offset = 0;
    uint32_t stamps_offset = 0;
    for (std::vector<sensor_msgs::msg::PointField>::iterator f = cloud_out.fields.begin();
      f != cloud_out.fields.end(); f++)
    {
      if (f->name == "x") {x_offset = f->offset;}
      if (f->name == "y") {y_offset = f->offset;}
      if (f->name == "z") {z_offset = f->offset;}
      if (f->name == "intensity") {intensity_offset = f->offset;}
      if (f->name == "distances") {distance_offset = f->offset;}
      if (f->name == "stamps") {stamps_offset = f->offset;}
    }

    for (unsigned int i = 0; i < cloud_out.width; i++) {
      EXPECT_NEAR(
        cloudData<float>(cloud_out, i * cloud_out.point_step + x_offset),
        static_cast<float>(static_cast<double>(scan.ranges[i]) *
        cos(static_cast<double>(scan.angle_min) + i * static_cast<double>(scan.angle_increment))),
        tolerance);
      EXPECT_NEAR(
        cloudData<float>(cloud_out, i * cloud_out.point_step + y_offset),
        static_cast<float>(static_cast<double>(scan.ranges[i]) *
        sin(static_cast<double>(scan.angle_min) + i * static_cast<double>(scan.angle_increment))),
        tolerance);
      EXPECT_NEAR(cloudData<float>(cloud_out, i * cloud_out.point_step + z_offset), 0, tolerance);
      EXPECT_NEAR(
        cloudData<float>(cloud_out, i * cloud_out.point_step + intensity_offset),
        scan.intensities[i], tolerance);  // intensity
      EXPECT_NEAR(
        cloudData<float>(cloud_out, i * cloud_out.point_step + distance_offset),
        scan.ranges[i], tolerance);  // ranges
      EXPECT_NEAR(
        cloudData<float>(cloud_out, i * cloud_out.point_step + stamps_offset),
        (float)i * scan.time_increment, tolerance);  // timestamps
    }
  }
}

TEST(laser_geometry, transformLaserScanToPointCloudEmptyScan) {
  tf2::BufferCore tf_buffer;

  geometry_msgs::msg::TransformStamped t_laser;
  t_laser.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  t_laser.header.frame_id = "world";
  t_laser.child_frame_id = "laser_frame";
  t_laser.transform.translation.x = 1.0;
  t_laser.transform.rotation.w = 1.0;
  tf_buffer.setTransform(t_laser, "test_authority", false);

  geometry_msgs::msg::TransformStamped t_base;
  t_base.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  t_base.header.frame_id = "world";
  t_base.child_frame_id = "base_link";
  t_base.transform.translation.x = 2.0;
  t_base.transform.rotation.w = 1.0;
  tf_buffer.setTransform(t_base, "test_authority", false);

  laser_geometry::LaserProjection projector;

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser_frame";
  scan.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  scan.angle_min = 0.0f;
  scan.angle_max = 0.0f;
  scan.angle_increment = 0.0f;
  scan.time_increment = 0.0f;
  scan.scan_time = 0.1f;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;

  sensor_msgs::msg::PointCloud2 cloud_out;

  // Empty scan with finite time_increment and target frame "world"
  EXPECT_NO_THROW(projector.transformLaserScanToPointCloud("world", scan, cloud_out, tf_buffer));
  ASSERT_EQ(cloud_out.width, 0u);

  // Empty scan with infinite time_increment and different frame "base_link"
  scan.time_increment = std::numeric_limits<float>::infinity();
  EXPECT_NO_THROW(
    projector.transformLaserScanToPointCloud("base_link", scan, cloud_out, tf_buffer));
  ASSERT_EQ(cloud_out.width, 0u);

  // Empty scan with infinite time_increment and same frame "laser_frame"
  EXPECT_NO_THROW(
    projector.transformLaserScanToPointCloud("laser_frame", scan, cloud_out, tf_buffer));
  ASSERT_EQ(cloud_out.width, 0u);
}

TEST(laser_geometry, transformLaserScanToPointCloudEmptyScanWithoutTransform) {
  // No transforms have been published, but an empty scan has nothing to transform
  tf2::BufferCore tf_buffer;
  laser_geometry::LaserProjection projector;

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser_frame";
  scan.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  scan.time_increment = std::numeric_limits<float>::infinity();
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;

  sensor_msgs::msg::PointCloud2 cloud_out;
  EXPECT_NO_THROW(
    projector.transformLaserScanToPointCloud(
      "world", scan, cloud_out, tf_buffer, -1.0, laser_geometry::channel_option::Intensity));
  ASSERT_EQ(cloud_out.width, 0u);
  EXPECT_EQ(cloud_out.header.frame_id, "world");
  EXPECT_TRUE(cloud_out.data.empty());
  for (const auto & f : cloud_out.fields) {
    EXPECT_NE(f.name, "index");
  }

  // A scan with rays still requires the transform
  scan.ranges.push_back(2.5f);
  scan.intensities.push_back(1.0f);
  EXPECT_THROW(
    projector.transformLaserScanToPointCloud("world", scan, cloud_out, tf_buffer),
    tf2::TransformException);
}

TEST(laser_geometry, projectLaserSingleRayTimestamp) {
  laser_geometry::LaserProjection projector;

  sensor_msgs::msg::LaserScan scan;
  scan.angle_increment = std::numeric_limits<float>::quiet_NaN();
  scan.time_increment = std::numeric_limits<float>::infinity();
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges.push_back(2.5f);

  sensor_msgs::msg::PointCloud2 cloud_out;
  projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Timestamp);
  ASSERT_EQ(cloud_out.width, 1u);

  uint32_t stamps_offset = 0;
  for (const auto & f : cloud_out.fields) {
    if (f.name == "stamps") {stamps_offset = f.offset;}
  }
  ASSERT_GT(stamps_offset, 0u);
  EXPECT_FLOAT_EQ(cloudData<float>(cloud_out, stamps_offset), 0.0f);
}

TEST(laser_geometry, projectLaserRejectsNonFiniteAngleIncrement) {
  laser_geometry::LaserProjection projector;

  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0.0f;
  scan.angle_max = 1.0f;
  scan.angle_increment = std::numeric_limits<float>::quiet_NaN();
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges = {2.0f, 2.0f, 2.0f};

  sensor_msgs::msg::PointCloud2 cloud_out;
  EXPECT_THROW(projector.projectLaser(scan, cloud_out, -1.0, 0), std::invalid_argument);

  scan.angle_increment = std::numeric_limits<float>::infinity();
  EXPECT_THROW(projector.projectLaser(scan, cloud_out, -1.0, 0), std::invalid_argument);

  // The rejected scans must not leave invalid angles in the cached co_sine_map
  scan.angle_increment = 0.5f;
  projector.projectLaser(scan, cloud_out, -1.0, 0);
  ASSERT_EQ(cloud_out.width, 3u);
  EXPECT_NEAR(cloudData<float>(cloud_out, 2 * cloud_out.point_step), 2.0f * std::cos(1.0f), 1e-5);
}

TEST(laser_geometry, transformLaserScanToPointCloudSingleRay) {
  tf2::BufferCore tf_buffer;

  geometry_msgs::msg::TransformStamped t_start;
  t_start.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  t_start.header.frame_id = "world";
  t_start.child_frame_id = "laser_frame";
  t_start.transform.translation.x = 1.0;
  t_start.transform.rotation.w = 1.0;
  tf_buffer.setTransform(t_start, "test_authority", false);

  geometry_msgs::msg::TransformStamped t_mid;
  t_mid.header.stamp = rclcpp::Time(11, 0, RCL_ROS_TIME);
  t_mid.header.frame_id = "world";
  t_mid.child_frame_id = "laser_frame";
  t_mid.transform.translation.x = 5.0;
  t_mid.transform.rotation.w = 1.0;
  tf_buffer.setTransform(t_mid, "test_authority", false);

  laser_geometry::LaserProjection projector;

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser_frame";
  scan.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  scan.angle_min = 0.0f;
  scan.angle_max = 0.0f;
  scan.angle_increment = std::numeric_limits<float>::quiet_NaN();
  scan.time_increment = std::numeric_limits<float>::infinity();
  scan.scan_time = 0.1f;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges.push_back(2.5f);
  scan.intensities.push_back(1.0f);

  sensor_msgs::msg::PointCloud2 cloud_out;
  EXPECT_NO_THROW(projector.transformLaserScanToPointCloud("world", scan, cloud_out, tf_buffer));

  ASSERT_EQ(cloud_out.width, 1u);

  uint32_t x_offset = 0;
  uint32_t y_offset = 0;
  uint32_t z_offset = 0;
  for (const auto & f : cloud_out.fields) {
    if (f.name == "x") {x_offset = f.offset;}
    if (f.name == "y") {y_offset = f.offset;}
    if (f.name == "z") {z_offset = f.offset;}
  }

  // Single-ray scan at stamp 10.0 with range 2.5: transforms with start transform (tx=1.0)
  // producing 3.5 without NaN coordinates (Issue #73).
  EXPECT_NEAR(cloudData<float>(cloud_out, x_offset), 3.5f, 1e-6);
  EXPECT_NEAR(cloudData<float>(cloud_out, y_offset), 0.0f, 1e-6);
  EXPECT_NEAR(cloudData<float>(cloud_out, z_offset), 0.0f, 1e-6);

  // Verify multi-ray scan correctly interpolates between stamp and stamp + time_increment
  sensor_msgs::msg::LaserScan scan_two_rays = scan;
  scan_two_rays.angle_increment = 0.0f;
  scan_two_rays.time_increment = 1.0f;
  scan_two_rays.ranges.push_back(2.5f);
  scan_two_rays.intensities.push_back(1.0f);

  sensor_msgs::msg::PointCloud2 cloud_two_rays;
  projector.transformLaserScanToPointCloud("world", scan_two_rays, cloud_two_rays, tf_buffer);

  ASSERT_EQ(cloud_two_rays.width, 2u);
  // First ray (ratio = 0.0) at stamp 10.0 (tx=1.0): 1.0 + 2.5 = 3.5
  EXPECT_NEAR(
    cloudData<float>(cloud_two_rays, 0 * cloud_two_rays.point_step + x_offset),
    3.5f, 1e-6);
  // Second ray (ratio = 1.0) at stamp 11.0 (tx=5.0): 5.0 + 2.5 = 7.5
  EXPECT_NEAR(
    cloudData<float>(cloud_two_rays, 1 * cloud_two_rays.point_step + x_offset),
    7.5f, 1e-6);
}

TEST(laser_geometry, transformLaserScanToPointCloudInterpolationWithRotation) {
  tf2::BufferCore tf_buffer;

  geometry_msgs::msg::TransformStamped t_start;
  t_start.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  t_start.header.frame_id = "world";
  t_start.child_frame_id = "laser_frame";
  t_start.transform.translation.x = 1.0;
  t_start.transform.rotation.w = 1.0;
  tf_buffer.setTransform(t_start, "test_authority", false);

  geometry_msgs::msg::TransformStamped t_end;
  t_end.header.stamp = rclcpp::Time(12, 0, RCL_ROS_TIME);
  t_end.header.frame_id = "world";
  t_end.child_frame_id = "laser_frame";
  t_end.transform.translation.x = 10.0;
  // Rotation around Z by pi/2 (yaw = pi/2)
  t_end.transform.rotation.z = std::sin(M_PI / 4.0);
  t_end.transform.rotation.w = std::cos(M_PI / 4.0);
  tf_buffer.setTransform(t_end, "test_authority", false);

  laser_geometry::LaserProjection projector;

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser_frame";
  scan.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  scan.angle_min = 0.0f;
  scan.angle_max = 0.0f;
  scan.angle_increment = 0.0f;
  scan.time_increment = 1.0f;
  scan.scan_time = 2.0f;
  scan.range_min = 0.1f;
  scan.range_max = 15.0f;
  scan.ranges = {2.0f, 2.0f, 2.0f};
  scan.intensities = {1.0f, 1.0f, 1.0f};

  sensor_msgs::msg::PointCloud2 cloud;
  projector.transformLaserScanToPointCloud("world", scan, cloud, tf_buffer);

  ASSERT_EQ(cloud.width, 3u);

  uint32_t x_offset = 0;
  uint32_t y_offset = 0;
  uint32_t z_offset = 0;
  for (const auto & f : cloud.fields) {
    if (f.name == "x") {x_offset = f.offset;}
    if (f.name == "y") {y_offset = f.offset;}
    if (f.name == "z") {z_offset = f.offset;}
  }

  const uint32_t step = cloud.point_step;
  const float sqrt2 = static_cast<float>(std::sqrt(2.0));

  // Ray 0 at t=10.0s (ratio = 0.0):
  // T=(1.0, 0, 0), yaw=0 -> x = 1.0 + 2.0 = 3.0, y = 0.0, z = 0.0
  EXPECT_NEAR(cloudData<float>(cloud, 0 * step + x_offset), 3.0f, 1e-5);
  EXPECT_NEAR(cloudData<float>(cloud, 0 * step + y_offset), 0.0f, 1e-5);
  EXPECT_NEAR(cloudData<float>(cloud, 0 * step + z_offset), 0.0f, 1e-5);

  // Ray 1 at t=11.0s (ratio = 0.5):
  // Verifies intermediate interpolation at ratio 1 / (N - 1) = 0.5
  // where tx = 5.5 (not 1/N = 1/3 -> tx = 4.0).
  // Slerp gives yaw = pi/4: x = 5.5 + 2*cos(pi/4) = 5.5 + sqrt(2),
  // y = 2*sin(pi/4) = sqrt(2).
  EXPECT_NEAR(cloudData<float>(cloud, 1 * step + x_offset), 5.5f + sqrt2, 1e-5);
  EXPECT_NEAR(cloudData<float>(cloud, 1 * step + y_offset), sqrt2, 1e-5);
  EXPECT_NEAR(cloudData<float>(cloud, 1 * step + z_offset), 0.0f, 1e-5);

  // Ray 2 at t=12.0s (ratio = 1.0):
  // T=(10.0, 0, 0), yaw=pi/2 -> x = 10.0, y = 2.0, z = 0.0
  EXPECT_NEAR(cloudData<float>(cloud, 2 * step + x_offset), 10.0f, 1e-5);
  EXPECT_NEAR(cloudData<float>(cloud, 2 * step + y_offset), 2.0f, 1e-5);
  EXPECT_NEAR(cloudData<float>(cloud, 2 * step + z_offset), 0.0f, 1e-5);
}

TEST(laser_geometry, projectLaserAngleIncrementChange) {
  laser_geometry::LaserProjection projector;

  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0.0f;
  scan.angle_max = 1.0f;
  scan.angle_increment = 0.0f;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges = {2.0f, 2.0f, 2.0f};

  sensor_msgs::msg::PointCloud2 cloud_out;
  projector.projectLaser(scan, cloud_out, -1.0, 0);
  ASSERT_EQ(cloud_out.width, 3u);
  EXPECT_NEAR(cloudData<float>(cloud_out, 2 * cloud_out.point_step), 2.0f, 1e-5);

  // Only the increment changes, so the cached co_sine_map must still be recomputed
  scan.angle_increment = 0.5f;
  projector.projectLaser(scan, cloud_out, -1.0, 0);
  ASSERT_EQ(cloud_out.width, 3u);
  EXPECT_NEAR(cloudData<float>(cloud_out, 2 * cloud_out.point_step), 2.0f * std::cos(1.0f), 1e-5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
