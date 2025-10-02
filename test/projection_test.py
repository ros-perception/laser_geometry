# Copyright (c) 2014, Enrique Fernandez
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from itertools import product

from laser_geometry import LaserProjection
import numpy as np
import pytest
import rclpy
import rclpy.duration
import rclpy.time
from sensor_msgs.msg import LaserScan
import sensor_msgs_py.point_cloud2 as pc2

PROJECTION_TEST_RANGE_MIN = 0.23
PROJECTION_TEST_RANGE_MAX = 40.00


class BuildScanException(Exception):
    pass


def build_constant_scan(
        range_val, intensity_val,
        angle_min, angle_max, angle_increment, scan_time):
    count = np.uint(np.ceil((angle_max - angle_min) / angle_increment))
    if count < 0:
        raise BuildScanException

    scan = LaserScan()
    scan.header.stamp = rclpy.time.Time(seconds=10.10).to_msg()
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_increment
    scan.scan_time = scan_time.nanoseconds / 1e9
    scan.range_min = PROJECTION_TEST_RANGE_MIN
    scan.range_max = PROJECTION_TEST_RANGE_MAX
    scan.ranges = [range_val for _ in range(count)]
    scan.intensities = [intensity_val for _ in range(count)]
    scan.time_increment = scan_time.nanoseconds / 1e9 / count

    return scan


def test_project_laser():
    tolerance = 6  # decimal places
    projector = LaserProjection()

    ranges = [-1.0, 1.0, 5.0, 100.0]
    intensities = np.arange(1.0, 4.0).tolist()

    min_angles = -np.pi / np.array([1.0, 1.5, 8.0])
    max_angles = -min_angles

    angle_increments = np.pi / np.array([180., 360., 720.])

    scan_times = [rclpy.duration.Duration(seconds=1./i) for i in [40, 20]]

    for range_val, intensity_val, \
        angle_min, angle_max, angle_increment, scan_time in \
        product(
            ranges, intensities,
            min_angles, max_angles,
            angle_increments, scan_times):
        try:
            scan = build_constant_scan(
                range_val, intensity_val,
                angle_min, angle_max, angle_increment, scan_time)
        except BuildScanException:
            assert (angle_max - angle_min)/angle_increment <= 0

        cloud_out = projector.projectLaser(
            scan, -1.0,
            LaserProjection.ChannelOption.INTENSITY |
            LaserProjection.ChannelOption.INDEX |
            LaserProjection.ChannelOption.DISTANCE |
            LaserProjection.ChannelOption.TIMESTAMP)
        assert len(cloud_out.fields) == 7, 'PointCloud2 with channel INDEX: fields size != 7'

        valid_points = 0
        for i in range(len(scan.ranges)):
            ri = scan.ranges[i]
            if (PROJECTION_TEST_RANGE_MIN <= ri and ri <= PROJECTION_TEST_RANGE_MAX):
                valid_points += 1

        assert valid_points == cloud_out.width, 'Valid points != PointCloud2 width'

        idx_x = idx_y = idx_z = 0
        idx_intensity = idx_index = 0
        idx_distance = idx_stamps = 0

        i = 0
        for f in cloud_out.fields:
            if f.name == 'x':
                idx_x = i
            elif f.name == 'y':
                idx_y = i
            elif f.name == 'z':
                idx_z = i
            elif f.name == 'intensity':
                idx_intensity = i
            elif f.name == 'index':
                idx_index = i
            elif f.name == 'distances':
                idx_distance = i
            elif f.name == 'stamps':
                idx_stamps = i
            i += 1

        i = 0
        for point in pc2.read_points(cloud_out):
            ri = scan.ranges[i]
            ai = scan.angle_min + i * scan.angle_increment

            assert point[idx_x] == pytest.approx(ri * np.cos(ai), abs=tolerance), 'x not equal'
            assert point[idx_y] == pytest.approx(ri * np.sin(ai), tolerance), 'y not equal'
            assert point[idx_z] == pytest.approx(0, tolerance), 'z not equal'
            assert point[idx_intensity] == pytest.approx(
                scan.intensities[i],
                tolerance), 'Intensity not equal'
            assert point[idx_index] == pytest.approx(i, tolerance), 'Index not equal'
            assert point[idx_distance] == pytest.approx(ri, tolerance), 'Distance not equal'
            assert point[idx_stamps] == pytest.approx(
                i * scan.time_increment, tolerance), 'Timestamp not equal'
            i += 1


if __name__ == '__main__':
    pytest.main()
