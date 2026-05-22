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

#include "laser_geometry/laser_geometry.hpp"

#include <Eigen/Core>

#include <string>

#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace laser_geometry
{
void LaserProjection::projectLaser_(
  const sensor_msgs::msg::LaserScan & scan_in,
  sensor_msgs::msg::PointCloud2 & cloud_out,
  double range_cutoff,
  int channel_options)
{
  size_t n_pts = scan_in.ranges.size();
  Eigen::ArrayXd ranges(n_pts);

  for (size_t i = 0; i < n_pts; ++i) {
    ranges(i) = static_cast<double>(scan_in.ranges[i]);
  }

  // Check if our existing co_sine_map is valid
  if (co_sine_map_.rows() != static_cast<int>(n_pts) || angle_min_ != scan_in.angle_min ||
    angle_max_ != scan_in.angle_max)
  {
    co_sine_map_ = Eigen::ArrayXXd(n_pts, 2);
    angle_min_ = scan_in.angle_min;
    angle_max_ = scan_in.angle_max;
    // Spherical->Cartesian projection
    for (size_t i = 0; i < n_pts; ++i) {
      co_sine_map_(i, 0) =
        cos(scan_in.angle_min + static_cast<double>(i) * scan_in.angle_increment);
      co_sine_map_(i, 1) =
        sin(scan_in.angle_min + static_cast<double>(i) * scan_in.angle_increment);
    }
  }

  Eigen::ArrayXXd output(n_pts, 2);
  output.col(0) = co_sine_map_.col(0) * ranges;
  output.col(1) = co_sine_map_.col(1) * ranges;

  // Set the output cloud accordingly
  cloud_out.header = scan_in.header;
  cloud_out.height = 1;
  cloud_out.width = static_cast<uint32_t>(scan_in.ranges.size());
  cloud_out.fields.resize(3);
  cloud_out.fields[0].name = "x";
  cloud_out.fields[0].offset = 0;
  cloud_out.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_out.fields[0].count = 1;
  cloud_out.fields[1].name = "y";
  cloud_out.fields[1].offset = 4;
  cloud_out.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_out.fields[1].count = 1;
  cloud_out.fields[2].name = "z";
  cloud_out.fields[2].offset = 8;
  cloud_out.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_out.fields[2].count = 1;

  // Define 4 indices in the channel array for each possible value type
  int idx_intensity = -1, idx_index = -1, idx_distance = -1, idx_timestamp = -1, idx_vpx = -1;

  // now, we need to check what fields we need to store
  uint32_t offset = 12;
  if ((channel_options & channel_option::Intensity) && scan_in.intensities.size() > 0) {
    size_t field_size = cloud_out.fields.size();
    cloud_out.fields.resize(field_size + 1);
    cloud_out.fields[field_size].name = "intensity";
    cloud_out.fields[field_size].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_out.fields[field_size].offset = offset;
    cloud_out.fields[field_size].count = 1;
    offset += 4;
    idx_intensity = static_cast<int>(field_size);
  }

  if ((channel_options & channel_option::Index)) {
    size_t field_size = cloud_out.fields.size();
    cloud_out.fields.resize(field_size + 1);
    cloud_out.fields[field_size].name = "index";
    cloud_out.fields[field_size].datatype = sensor_msgs::msg::PointField::INT32;
    cloud_out.fields[field_size].offset = offset;
    cloud_out.fields[field_size].count = 1;
    offset += 4;
    idx_index = static_cast<int>(field_size);
  }

  if ((channel_options & channel_option::Distance)) {
    size_t field_size = cloud_out.fields.size();
    cloud_out.fields.resize(field_size + 1);
    cloud_out.fields[field_size].name = "distances";
    cloud_out.fields[field_size].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_out.fields[field_size].offset = offset;
    cloud_out.fields[field_size].count = 1;
    offset += 4;
    idx_distance = static_cast<int>(field_size);
  }

  if ((channel_options & channel_option::Timestamp)) {
    size_t field_size = cloud_out.fields.size();
    cloud_out.fields.resize(field_size + 1);
    cloud_out.fields[field_size].name = "stamps";
    cloud_out.fields[field_size].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_out.fields[field_size].offset = offset;
    cloud_out.fields[field_size].count = 1;
    offset += 4;
    idx_timestamp = static_cast<int>(field_size);
  }

  if ((channel_options & channel_option::Viewpoint)) {
    size_t field_size = cloud_out.fields.size();
    cloud_out.fields.resize(field_size + 3);

    cloud_out.fields[field_size].name = "vp_x";
    cloud_out.fields[field_size].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_out.fields[field_size].offset = offset;
    cloud_out.fields[field_size].count = 1;
    offset += 4;

    cloud_out.fields[field_size + 1].name = "vp_y";
    cloud_out.fields[field_size + 1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_out.fields[field_size + 1].offset = offset;
    cloud_out.fields[field_size + 1].count = 1;
    offset += 4;

    cloud_out.fields[field_size + 2].name = "vp_z";
    cloud_out.fields[field_size + 2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_out.fields[field_size + 2].offset = offset;
    cloud_out.fields[field_size + 2].count = 1;
    offset += 4;

    idx_vpx = static_cast<int>(field_size);
  }

  cloud_out.point_step = offset;
  cloud_out.row_step = cloud_out.point_step * cloud_out.width;
  cloud_out.data.resize(cloud_out.row_step * cloud_out.height);
  cloud_out.is_dense = false;

  if (range_cutoff < 0) {
    range_cutoff = scan_in.range_max;
  }

  unsigned int count = 0;
  for (size_t i = 0; i < n_pts; ++i) {
    // check to see if we want to keep the point
    const float range = scan_in.ranges[i];
    if (range < range_cutoff && range >= scan_in.range_min) {
      auto pstep = reinterpret_cast<float *>(&cloud_out.data[count * cloud_out.point_step]);

      // Copy XYZ
      pstep[0] = static_cast<float>(output(i, 0));
      pstep[1] = static_cast<float>(output(i, 1));
      pstep[2] = 0;

      // Copy intensity
      if (idx_intensity != -1) {
        pstep[idx_intensity] = scan_in.intensities[i];
      }

      // Copy index
      if (idx_index != -1) {
        reinterpret_cast<int *>(pstep)[idx_index] = static_cast<int>(i);
      }

      // Copy distance
      if (idx_distance != -1) {
        pstep[idx_distance] = range;
      }

      // Copy timestamp
      if (idx_timestamp != -1) {
        pstep[idx_timestamp] = i * scan_in.time_increment;
      }

      // Copy viewpoint (0, 0, 0)
      if (idx_vpx != -1) {
        pstep[idx_vpx] = 0;
        pstep[idx_vpx + 1] = 0;
        pstep[idx_vpx + 2] = 0;
      }

      // make sure to increment count
      ++count;
    }
  }

  // resize if necessary
  cloud_out.width = count;
  cloud_out.row_step = cloud_out.point_step * cloud_out.width;
  cloud_out.data.resize(cloud_out.row_step * cloud_out.height);
}

void LaserProjection::transformLaserScanToPointCloud_(
  const std::string & target_frame,
  const sensor_msgs::msg::LaserScan & scan_in,
  sensor_msgs::msg::PointCloud2 & cloud_out,
  tf2::BufferCore & tf,
  double range_cutoff,
  int channel_options)
{
  rclcpp::Time start_time(scan_in.header.stamp, RCL_ROS_TIME);
  rclcpp::Time end_time(scan_in.header.stamp, RCL_ROS_TIME);
  // TODO(anonymous): reconcile all the different time constructs
  if (!scan_in.ranges.empty()) {
    end_time = start_time + rclcpp::Duration::from_seconds(
      static_cast<double>(scan_in.ranges.size() - 1) * static_cast<double>(scan_in.time_increment));
  }

  std::chrono::nanoseconds start(start_time.nanoseconds());
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> st(start);
  geometry_msgs::msg::TransformStamped start_transform = tf.lookupTransform(
    target_frame, scan_in.header.frame_id, st);
  std::chrono::nanoseconds end(end_time.nanoseconds());
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> e(end);
  geometry_msgs::msg::TransformStamped end_transform = tf.lookupTransform(
    target_frame, scan_in.header.frame_id, e);

  tf2::Quaternion quat_start;
  tf2::Quaternion quat_end;
  tf2::fromMsg(start_transform.transform.rotation, quat_start);
  tf2::fromMsg(end_transform.transform.rotation, quat_end);

  tf2::Vector3 origin_start;
  tf2::Vector3 origin_end;
  tf2::fromMsg(start_transform.transform.translation, origin_start);
  tf2::fromMsg(end_transform.transform.translation, origin_end);

  // check if the user has requested the index field
  const bool requested_index = (channel_options & channel_option::Index) != 0;

  // we'll enforce that we get index values for the laser scan so that we
  // ensure that we use the correct timestamps
  channel_options |= channel_option::Index;

  projectLaser_(scan_in, cloud_out, range_cutoff, channel_options);

  // we'll assume no associated viewpoint by default
  bool has_viewpoint = false;
  uint32_t vp_x_offset = 0;

  // we need to find the offset of the intensity field in the point cloud
  // we also know that the index field is guaranteed to exist since we
  // set the channel option above. To be really safe, it might be worth
  // putting in a check at some point, but I'm just going to put in an
  // assert for now
  uint32_t index_offset = 0;
  for (unsigned int i = 0; i < cloud_out.fields.size(); ++i) {
    if (cloud_out.fields[i].name == "index") {
      index_offset = cloud_out.fields[i].offset;
    }

    // we want to check if the cloud has a viewpoint associated with it
    // checking vp_x should be sufficient since vp_x, vp_y, and vp_z all
    // get put in together
    if (cloud_out.fields[i].name == "vp_x") {
      has_viewpoint = true;
      vp_x_offset = cloud_out.fields[i].offset;
    }
  }

  assert(index_offset > 0);

  cloud_out.header.frame_id = target_frame;

  tf2::Transform cur_transform;

  double ranges_norm = 1 / (static_cast<double>(scan_in.ranges.size()) - 1.0);

  // we want to loop through all the points in the cloud
  for (size_t i = 0; i < cloud_out.width; ++i) {
    // Apply the transform to the current point
    float * pstep = reinterpret_cast<float *>(&cloud_out.data[i * cloud_out.point_step + 0]);

    // find the index of the point
    uint32_t pt_index;
    memcpy(&pt_index, &cloud_out.data[i * cloud_out.point_step + index_offset], sizeof(uint32_t));

    // Assume constant motion during the laser-scan and use slerp to compute intermediate transforms
    double ratio = pt_index * ranges_norm;

    // TODO(anon): Make a function that performs both the slerp and linear interpolation needed to
    // interpolate a Full Transform (Quaternion + Vector)
    // Interpolate translation
    tf2::Vector3 v(0, 0, 0);
    v.setInterpolate3(origin_start, origin_end, ratio);
    cur_transform.setOrigin(v);

    // Compute the slerp-ed rotation
    cur_transform.setRotation(slerp(quat_start, quat_end, ratio));

    tf2::Vector3 point_in(pstep[0], pstep[1], pstep[2]);
    tf2::Vector3 point_out = cur_transform * point_in;

    // Copy transformed point into cloud
    pstep[0] = static_cast<float>(point_out.x());
    pstep[1] = static_cast<float>(point_out.y());
    pstep[2] = static_cast<float>(point_out.z());

    // Convert the viewpoint as well
    if (has_viewpoint) {
      auto vpstep =
        reinterpret_cast<float *>(&cloud_out.data[i * cloud_out.point_step + vp_x_offset]);
      point_in = tf2::Vector3(vpstep[0], vpstep[1], vpstep[2]);
      point_out = cur_transform * point_in;

      // Copy transformed point into cloud
      vpstep[0] = static_cast<float>(point_out.x());
      vpstep[1] = static_cast<float>(point_out.y());
      vpstep[2] = static_cast<float>(point_out.z());
    }
  }

  // if the user didn't request the index field, strip the one we added internally
  if (!requested_index) {
    std::vector<sensor_msgs::msg::PointField> new_fields;
    new_fields.reserve(cloud_out.fields.size() - 1);
    for (const auto & f : cloud_out.fields) {
      if (f.name == "index") {continue;}
      new_fields.push_back(f);
      if (new_fields.back().offset > index_offset) {
        new_fields.back().offset -= 4;
      }
    }

    const uint32_t old_point_step = cloud_out.point_step;
    const uint32_t new_point_step = old_point_step - 4;
    const uint32_t tail_offset = index_offset + 4;
    const uint32_t tail_size = old_point_step - tail_offset;

    std::vector<uint8_t> new_data(static_cast<size_t>(new_point_step) * cloud_out.width);
    for (uint32_t i = 0; i < cloud_out.width; ++i) {
      const uint8_t * src = &cloud_out.data[i * old_point_step];
      uint8_t * dst = &new_data[i * new_point_step];
      memcpy(dst, src, index_offset);
      if (tail_size > 0) {
        memcpy(dst + index_offset, src + tail_offset, tail_size);
      }
    }

    cloud_out.fields = std::move(new_fields);
    cloud_out.point_step = new_point_step;
    cloud_out.row_step = new_point_step * cloud_out.width;
    cloud_out.data = std::move(new_data);
  }
}

void LaserProjection::transformLaserScanToPointCloud_(
  const std::string & target_frame,
  const sensor_msgs::msg::LaserScan & scan_in,
  sensor_msgs::msg::PointCloud2 & cloud_out,
  tf2::BufferCore & tf,
  double range_cutoff,
  int channel_options)
{
  rclcpp::Time start_time(scan_in.header.stamp, RCL_ROS_TIME);
  rclcpp::Time end_time(scan_in.header.stamp, RCL_ROS_TIME);
  // TODO(anonymous): reconcile all the different time constructs
  if (!scan_in.ranges.empty()) {
    end_time = start_time + rclcpp::Duration::from_seconds(
      static_cast<double>(scan_in.ranges.size() - 1) * static_cast<double>(scan_in.time_increment));
  }

  std::chrono::nanoseconds start(start_time.nanoseconds());
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> st(start);
  geometry_msgs::msg::TransformStamped start_transform = tf.lookupTransform(
    target_frame, scan_in.header.frame_id, st);
  std::chrono::nanoseconds end(end_time.nanoseconds());
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> e(end);
  geometry_msgs::msg::TransformStamped end_transform = tf.lookupTransform(
    target_frame, scan_in.header.frame_id, e);

  tf2::Quaternion quat_start;
  tf2::Quaternion quat_end;
  tf2::fromMsg(start_transform.transform.rotation, quat_start);
  tf2::fromMsg(end_transform.transform.rotation, quat_end);

  tf2::Vector3 origin_start;
  tf2::Vector3 origin_end;
  tf2::fromMsg(start_transform.transform.translation, origin_start);
  tf2::fromMsg(end_transform.transform.translation, origin_end);
  transformLaserScanToPointCloud_(
    target_frame, scan_in, cloud_out,
    quat_start, origin_start,
    quat_end, origin_end,
    range_cutoff,
    channel_options);
}

}  // namespace laser_geometry
