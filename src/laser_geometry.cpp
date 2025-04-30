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

#include <algorithm>
#include <string>

#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/LinearMath/Transform.hpp"

namespace laser_geometry
{
void LaserProjection::projectLaser_(
  const sensor_msgs::msg::LaserScan & scan_in,
  sensor_msgs::msg::PointCloud2 & cloud_out,
  double range_cutoff,
  int channel_options)
{
  size_t n_pts = scan_in.ranges.size();
  Eigen::ArrayXXd ranges(n_pts, 2);
  Eigen::ArrayXXd output(n_pts, 2);

  // Get the ranges into Eigen format
  for (size_t i = 0; i < n_pts; ++i) {
    ranges(i, 0) = static_cast<double>(scan_in.ranges[i]);
    ranges(i, 1) = static_cast<double>(scan_in.ranges[i]);
  }

  // Check if our existing co_sine_map is valid
  if (co_sine_map_.rows() != static_cast<int>(n_pts) || angle_min_ != scan_in.angle_min ||
    angle_max_ != scan_in.angle_max)
  {
    // ROS_DEBUG("[projectLaser] No precomputed map given. Computing one.");
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

  output = ranges * co_sine_map_;

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
  int idx_intensity = -1, idx_index = -1, idx_distance = -1, idx_timestamp = -1, idx_vpx = -1,
    idx_vpy = -1, idx_vpz = -1;

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
    idx_vpy = static_cast<int>(field_size + 1);
    idx_vpz = static_cast<int>(field_size + 2);
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
      if (idx_vpx != -1 && idx_vpy != -1 && idx_vpz != -1) {
        pstep[idx_vpx] = 0;
        pstep[idx_vpy] = 0;
        pstep[idx_vpz] = 0;
      }

      // make sure to increment count
      ++count;
    }

    /* TODO(anonymous): Why was this done in this way, I don't get this at all, you end up with a
     * ton of points with NaN values why can't you just leave them out?
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

  // resize if necessary
  cloud_out.width = count;
  cloud_out.row_step = cloud_out.point_step * cloud_out.width;
  cloud_out.data.resize(cloud_out.row_step * cloud_out.height);
}

void LaserProjection::transformLaserScanToPointCloud_(
  const std::string & target_frame,
  const sensor_msgs::msg::LaserScan & scan_in,
  sensor_msgs::msg::PointCloud2 & cloud_out,
  tf2::Quaternion quat_start,
  tf2::Vector3 origin_start,
  tf2::Quaternion quat_end,
  tf2::Vector3 origin_end,
  double range_cutoff,
  int channel_options)
{
  // check if the user has requested the index field
  bool requested_index = false;
  if ((channel_options & channel_option::Index)) {
    requested_index = true;
  }

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

  // if the user didn't request the index field, then we need to copy the PointCloud and drop it
  if (!requested_index) {
    sensor_msgs::msg::PointCloud2 cloud_without_index;

    // copy basic meta data
    cloud_without_index.header = cloud_out.header;
    cloud_without_index.width = cloud_out.width;
    cloud_without_index.height = cloud_out.height;
    cloud_without_index.is_bigendian = cloud_out.is_bigendian;
    cloud_without_index.is_dense = cloud_out.is_dense;

    // copy the fields
    cloud_without_index.fields.resize(cloud_out.fields.size());
    unsigned int field_count = 0;
    unsigned int offset_shift = 0;
    for (unsigned int i = 0; i < cloud_out.fields.size(); ++i) {
      if (cloud_out.fields[i].name != "index") {
        cloud_without_index.fields[field_count] = cloud_out.fields[i];
        cloud_without_index.fields[field_count].offset -= offset_shift;
        ++field_count;
      } else {
        // once we hit the index, we'll set the shift
        offset_shift = 4;
      }
    }

    // resize the fields
    cloud_without_index.fields.resize(field_count);

    // compute the size of the new data
    cloud_without_index.point_step = cloud_out.point_step - offset_shift;
    cloud_without_index.row_step = cloud_without_index.point_step * cloud_without_index.width;
    cloud_without_index.data.resize(cloud_without_index.row_step * cloud_without_index.height);

    uint32_t i = 0;
    uint32_t j = 0;
    // copy over the data from one cloud to the other
    while (i < cloud_out.data.size()) {
      if ((i % cloud_out.point_step) < index_offset ||
        (i % cloud_out.point_step) >= (index_offset + 4))
      {
        cloud_without_index.data[j++] = cloud_out.data[i];
      }
      i++;
    }

    // make sure to actually set the output
    cloud_out = cloud_without_index;
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
  rclcpp::Time start_time = scan_in.header.stamp;
  rclcpp::Time end_time = scan_in.header.stamp;
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
  transformLaserScanToPointCloud_(
    target_frame, scan_in, cloud_out,
    quat_start, origin_start,
    quat_end, origin_end,
    range_cutoff,
    channel_options);
}

}  // namespace laser_geometry
