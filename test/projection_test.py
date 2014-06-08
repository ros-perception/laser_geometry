#!/usr/bin/env python

PKG='laser_geometry'

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

import numpy as np
from itertools import product

import unittest

PROJECTION_TEST_RANGE_MIN =  0.23
PROJECTION_TEST_RANGE_MAX = 40.00

class BuildScanException:
    pass

def build_constant_scan(
        range_val, intensity_val,
        angle_min, angle_max, angle_increment, scan_time):
    count = np.uint(np.ceil((angle_max - angle_min) / angle_increment))
    if count < 0:
        raise BuildScanException

    scan = LaserScan()
    scan.header.stamp = rospy.rostime.Time.from_sec(10.10)
    scan.header.frame_id = "laser_frame"
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_increment
    scan.scan_time = scan_time.to_sec()
    scan.range_min = PROJECTION_TEST_RANGE_MIN
    scan.range_max = PROJECTION_TEST_RANGE_MAX
    scan.ranges = [range_val for _ in range(count)]
    scan.intensities = [intensity_val for _ in range(count)]
    scan.time_increment = scan_time.to_sec()/count

    return scan

class ProjectionTest(unittest.TestCase):

    def test_project_laser(self):
        tolerance = 6 # decimal places
        projector = LaserProjection()

        ranges = [-1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 100.0]
        intensities = np.arange(1.0, 6.0).tolist()

        min_angles = -np.pi / np.array([1.0, 1.5, 2.0, 4.0, 8.0])
        max_angles = -min_angles

        angle_increments = np.pi / np.array([180., 360., 720.])

        scan_times = [rospy.Duration(1./i) for i in [40, 20]]

        for range_val, intensity_val, \
            angle_min, angle_max, angle_increment, scan_time in \
            product(ranges, intensities,
                min_angles, max_angles, angle_increments, scan_times):
            try:
                scan = build_constant_scan(
                    range_val, intensity_val,
                    angle_min, angle_max, angle_increment, scan_time)
            except BuildScanException:
                if (angle_max - angle_min)/angle_increment > 0:
                    self.fail()

            cloud_out = projector.projectLaser(scan, -1.0,
                    LaserProjection.ChannelOption.INDEX)
            self.assertEquals(len(cloud_out.fields), 4,
                    "PointCloud2 with channel INDEX: fields size != 4")

            cloud_out = projector.projectLaser(scan, -1.0,
                    LaserProjection.ChannelOption.INTENSITY)
            self.assertEquals(len(cloud_out.fields), 4,
                    "PointCloud2 with channel INDEX: fields size != 4")

            cloud_out = projector.projectLaser(scan, -1.0)
            self.assertEquals(len(cloud_out.fields), 5,
                    "PointCloud2 with channel INDEX: fields size != 5")
            cloud_out = projector.projectLaser(scan, -1.0,
                    LaserProjection.ChannelOption.INTENSITY |
                    LaserProjection.ChannelOption.INDEX)
            self.assertEquals(len(cloud_out.fields), 5,
                    "PointCloud2 with channel INDEX: fields size != 5")

            cloud_out = projector.projectLaser(scan, -1.0,
                    LaserProjection.ChannelOption.INTENSITY |
                    LaserProjection.ChannelOption.INDEX |
                    LaserProjection.ChannelOption.DISTANCE)
            self.assertEquals(len(cloud_out.fields), 6,
                    "PointCloud2 with channel INDEX: fields size != 6")

            cloud_out = projector.projectLaser(scan, -1.0,
                    LaserProjection.ChannelOption.INTENSITY |
                    LaserProjection.ChannelOption.INDEX |
                    LaserProjection.ChannelOption.DISTANCE |
                    LaserProjection.ChannelOption.TIMESTAMP)
            self.assertEquals(len(cloud_out.fields), 7,
                    "PointCloud2 with channel INDEX: fields size != 7")

            valid_points = 0
            for i in range(len(scan.ranges)):
                ri = scan.ranges[i]
                if (PROJECTION_TEST_RANGE_MIN <= ri and
                    ri <= PROJECTION_TEST_RANGE_MAX):
                    valid_points += 1

            self.assertEqual(valid_points, cloud_out.width,
                    "Valid points != PointCloud2 width")

            idx_x = idx_y = idx_z = 0
            idx_intensity = idx_index = 0
            idx_distance = idx_stamps = 0

            i = 0
            for f in cloud_out.fields:
                if f.name == "x":
                    idx_x = i
                elif f.name == "y":
                    idx_y = i
                elif f.name == "z":
                    idx_z = i
                elif f.name == "intensity":
                    idx_intensity = i
                elif f.name == "index":
                    idx_index = i
                elif f.name == "distances":
                    idx_distance = i
                elif f.name == "stamps":
                    idx_stamps = i
                i += 1

            i = 0
            for point in pc2.read_points(cloud_out):
                ri = scan.ranges[i]
                ai = scan.angle_min + i * scan.angle_increment

                self.assertAlmostEqual(point[idx_x], ri * np.cos(ai),
                        tolerance, "x not equal")
                self.assertAlmostEqual(point[idx_y], ri * np.sin(ai),
                        tolerance, "y not equal")
                self.assertAlmostEqual(point[idx_z], 0,
                        tolerance, "z not equal")
                self.assertAlmostEqual(point[idx_intensity],
                        scan.intensities[i],
                        tolerance, "Intensity not equal")
                self.assertAlmostEqual(point[idx_index], i,
                        tolerance, "Index not equal")
                self.assertAlmostEqual(point[idx_distance], ri,
                        tolerance, "Distance not equal")
                self.assertAlmostEqual(point[idx_stamps],
                        i * scan.time_increment,
                        tolerance, "Timestamp not equal")
                i += 1


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'projection_test', ProjectionTest)
