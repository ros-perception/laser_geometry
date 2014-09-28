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

#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/PointCloud.h"
#include <math.h>


#include <angles/angles.h>

#include "rostest/permuter.h"

#define PROJECTION_TEST_RANGE_MIN (0.23)
#define PROJECTION_TEST_RANGE_MAX (40.0) 


class BuildScanException { };

sensor_msgs::LaserScan build_constant_scan(double range, double intensity, 
                                          double ang_min, double ang_max, double ang_increment,
                                          ros::Duration scan_time)
{
  if (((ang_max - ang_min) / ang_increment) < 0)
    throw (BuildScanException());

  sensor_msgs::LaserScan scan;
  scan.header.stamp = ros::Time::now();
  scan.header.frame_id = "laser_frame";
  scan.angle_min = ang_min;
  scan.angle_max = ang_max;
  scan.angle_increment = ang_increment;
  scan.scan_time = scan_time.toSec();
  scan.range_min = PROJECTION_TEST_RANGE_MIN;
  scan.range_max = PROJECTION_TEST_RANGE_MAX;
  uint32_t i = 0;
  for(; ang_min + i * ang_increment < ang_max; i++)
  {
    scan.ranges.push_back(range);
    scan.intensities.push_back(intensity);
  }

  scan.time_increment = scan_time.toSec()/(double)i;

  return scan;
};


class TestProjection : public laser_geometry::LaserProjection
{
public:
  const boost::numeric::ublas::matrix<double>& getUnitVectors(double angle_min,
                                                              double angle_max,
                                                              double angle_increment,
                                                              unsigned int length)
  {
    return getUnitVectors_(angle_min, angle_max, angle_increment, length);
  }
};

void test_getUnitVectors(double angle_min, double angle_max, double angle_increment, unsigned int length)
{
  double tolerance = 1e-12;
  TestProjection projector;  
  
  const boost::numeric::ublas::matrix<double> & mat = projector.getUnitVectors(angle_min, angle_max, angle_increment, length);
  
  

  for (unsigned int i = 0; i < mat.size2(); i++)
  {
    EXPECT_NEAR(angles::shortest_angular_distance(atan2(mat(1,i), mat(0,i)),
                                                   angle_min + i * angle_increment),
                0,
                tolerance); // check expected angle
    EXPECT_NEAR(1.0, mat(1,i)*mat(1,i) + mat(0,i)*mat(0,i), tolerance); //make sure normalized
  }
}

#if 0

TEST(laser_geometry, getUnitVectors)
{
  double min_angle = -M_PI/2;
  double max_angle = M_PI/2;
  double angle_increment = M_PI/180;

  std::vector<double> min_angles, max_angles, angle_increments;

  rostest::Permuter permuter;
  min_angles.push_back(-M_PI);
  min_angles.push_back(-M_PI/1.5);
  min_angles.push_back(-M_PI/2);
  min_angles.push_back(-M_PI/4);
  min_angles.push_back(-M_PI/8);
  min_angles.push_back(M_PI);
  min_angles.push_back(M_PI/1.5);
  min_angles.push_back(M_PI/2);
  min_angles.push_back(M_PI/4);
  min_angles.push_back(M_PI/8);
  permuter.addOptionSet(min_angles, &min_angle);

  max_angles.push_back(M_PI);
  max_angles.push_back(M_PI/1.5);
  max_angles.push_back(M_PI/2);
  max_angles.push_back(M_PI/4);
  max_angles.push_back(M_PI/8);
  max_angles.push_back(-M_PI);
  max_angles.push_back(-M_PI/1.5);
  max_angles.push_back(-M_PI/2);
  max_angles.push_back(-M_PI/4);
  max_angles.push_back(-M_PI/8);
  permuter.addOptionSet(max_angles, &max_angle);

  angle_increments.push_back(M_PI/180); // one degree
  angle_increments.push_back(M_PI/360); // half degree
  angle_increments.push_back(M_PI/720); // quarter degree
  angle_increments.push_back(-M_PI/180); // -one degree
  angle_increments.push_back(-M_PI/360); // -half degree
  angle_increments.push_back(-M_PI/720); // -quarter degree
  permuter.addOptionSet(angle_increments, &angle_increment);


  while (permuter.step())
  {
    if ((max_angle - min_angle) / angle_increment > 0.0)
    {
      unsigned int length = round((max_angle - min_angle)/ angle_increment);
      try
      {
        test_getUnitVectors(min_angle, max_angle, angle_increment, length);
        test_getUnitVectors(min_angle, max_angle, angle_increment, (max_angle - min_angle)/ angle_increment);
        test_getUnitVectors(min_angle, max_angle, angle_increment, (max_angle - min_angle)/ angle_increment + 1);
      }
      catch (BuildScanException &ex)
      {
      if ((max_angle - min_angle) / angle_increment > 0.0)//make sure it is not a false exception
        FAIL();
      }
    }
    //else 
      //printf("%f\n", (max_angle - min_angle) / angle_increment);
  }
}


TEST(laser_geometry, projectLaser)
{
  double tolerance = 1e-12;
  laser_geometry::LaserProjection projector;  

  double min_angle = -M_PI/2;
  double max_angle = M_PI/2;
  double angle_increment = M_PI/180;

  double range = 1.0;
  double intensity = 1.0;

  ros::Duration scan_time = ros::Duration(1/40);
  ros::Duration increment_time = ros::Duration(1/400);


  std::vector<double> ranges, intensities, min_angles, max_angles, angle_increments;
  std::vector<ros::Duration> increment_times, scan_times;

  rostest::Permuter permuter;

  ranges.push_back(-1.0);
  ranges.push_back(1.0);
  ranges.push_back(2.0);
  ranges.push_back(3.0);
  ranges.push_back(4.0);
  ranges.push_back(5.0);
  ranges.push_back(100.0);
  permuter.addOptionSet(ranges, &range);

  intensities.push_back(1.0);
  intensities.push_back(2.0);
  intensities.push_back(3.0);
  intensities.push_back(4.0);
  intensities.push_back(5.0);
  permuter.addOptionSet(intensities, &intensity);

  min_angles.push_back(-M_PI);
  min_angles.push_back(-M_PI/1.5);
  min_angles.push_back(-M_PI/2);
  min_angles.push_back(-M_PI/4);
  min_angles.push_back(-M_PI/8);
  permuter.addOptionSet(min_angles, &min_angle);

  max_angles.push_back(M_PI);
  max_angles.push_back(M_PI/1.5);
  max_angles.push_back(M_PI/2);
  max_angles.push_back(M_PI/4);
  max_angles.push_back(M_PI/8);
  permuter.addOptionSet(max_angles, &max_angle);

  //  angle_increments.push_back(-M_PI/180); // -one degree
  angle_increments.push_back(M_PI/180); // one degree
  angle_increments.push_back(M_PI/360); // half degree
  angle_increments.push_back(M_PI/720); // quarter degree
  permuter.addOptionSet(angle_increments, &angle_increment);

  scan_times.push_back(ros::Duration(1/40));
  scan_times.push_back(ros::Duration(1/20));

  permuter.addOptionSet(scan_times, &scan_time);

  while (permuter.step())
  {
    try
    {
      //      printf("%f %f %f %f %f %f\n", range, intensity, min_angle, max_angle, angle_increment, scan_time.toSec());
  sensor_msgs::LaserScan scan = build_constant_scan(range, intensity, min_angle, max_angle, angle_increment, scan_time);

  sensor_msgs::PointCloud cloud_out;
  projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Index);
  EXPECT_EQ(cloud_out.channels.size(), (unsigned int)1);
  projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Intensity);
  EXPECT_EQ(cloud_out.channels.size(), (unsigned int)1);

  projector.projectLaser(scan, cloud_out, -1.0);
  EXPECT_EQ(cloud_out.channels.size(), (unsigned int)2);
  projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index);
  EXPECT_EQ(cloud_out.channels.size(), (unsigned int)2);

  projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance);
  EXPECT_EQ(cloud_out.channels.size(), (unsigned int)3);

  projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance | laser_geometry::channel_option::Timestamp);
  EXPECT_EQ(cloud_out.channels.size(), (unsigned int)4);

  unsigned int valid_points = 0;
  for (unsigned int i = 0; i < scan.ranges.size(); i++)
    if (scan.ranges[i] <= PROJECTION_TEST_RANGE_MAX && scan.ranges[i] >= PROJECTION_TEST_RANGE_MIN)
      valid_points ++;

  EXPECT_EQ(valid_points, cloud_out.points.size());

  for (unsigned int i = 0; i < cloud_out.points.size(); i++)
  {
    EXPECT_NEAR(cloud_out.points[i].x , (float)((double)(scan.ranges[i]) * cos((double)(scan.angle_min) + i * (double)(scan.angle_increment))), tolerance);
    EXPECT_NEAR(cloud_out.points[i].y , (float)((double)(scan.ranges[i]) * sin((double)(scan.angle_min) + i * (double)(scan.angle_increment))), tolerance);
    EXPECT_NEAR(cloud_out.points[i].z, 0, tolerance);
    EXPECT_NEAR(cloud_out.channels[0].values[i], scan.intensities[i], tolerance);//intensity \todo determine this by lookup not hard coded order
    EXPECT_NEAR(cloud_out.channels[1].values[i], i, tolerance);//index
    EXPECT_NEAR(cloud_out.channels[2].values[i], scan.ranges[i], tolerance);//ranges
    EXPECT_NEAR(cloud_out.channels[3].values[i], (float)i * scan.time_increment, tolerance);//timestamps
  };
    }
    catch (BuildScanException &ex)
    {
        if ((max_angle - min_angle) / angle_increment > 0.0)//make sure it is not a false exception
            FAIL();
    }
  }
}

#endif


TEST(laser_geometry, projectLaser2)
{
  double tolerance = 1e-12;
  laser_geometry::LaserProjection projector;  

  double min_angle = -M_PI/2;
  double max_angle = M_PI/2;
  double angle_increment = M_PI/180;

  double range = 1.0;
  double intensity = 1.0;

  ros::Duration scan_time = ros::Duration(1/40);
  ros::Duration increment_time = ros::Duration(1/400);


  std::vector<double> ranges, intensities, min_angles, max_angles, angle_increments;
  std::vector<ros::Duration> increment_times, scan_times;

  rostest::Permuter permuter;

  ranges.push_back(-1.0);
  ranges.push_back(1.0);
  ranges.push_back(2.0);
  ranges.push_back(3.0);
  ranges.push_back(4.0);
  ranges.push_back(5.0);
  ranges.push_back(100.0);
  permuter.addOptionSet(ranges, &range);

  intensities.push_back(1.0);
  intensities.push_back(2.0);
  intensities.push_back(3.0);
  intensities.push_back(4.0);
  intensities.push_back(5.0);
  permuter.addOptionSet(intensities, &intensity);

  min_angles.push_back(-M_PI);
  min_angles.push_back(-M_PI/1.5);
  min_angles.push_back(-M_PI/2);
  min_angles.push_back(-M_PI/4);
  min_angles.push_back(-M_PI/8);
  permuter.addOptionSet(min_angles, &min_angle);

  max_angles.push_back(M_PI);
  max_angles.push_back(M_PI/1.5);
  max_angles.push_back(M_PI/2);
  max_angles.push_back(M_PI/4);
  max_angles.push_back(M_PI/8);
  permuter.addOptionSet(max_angles, &max_angle);

  //  angle_increments.push_back(-M_PI/180); // -one degree
  angle_increments.push_back(M_PI/180); // one degree
  angle_increments.push_back(M_PI/360); // half degree
  angle_increments.push_back(M_PI/720); // quarter degree
  permuter.addOptionSet(angle_increments, &angle_increment);

  scan_times.push_back(ros::Duration(1/40));
  scan_times.push_back(ros::Duration(1/20));

  permuter.addOptionSet(scan_times, &scan_time);

  while (permuter.step())
  {
    try
    {
      //        printf("%f %f %f %f %f %f\n", range, intensity, min_angle, max_angle, angle_increment, scan_time.toSec());
  sensor_msgs::LaserScan scan = build_constant_scan(range, intensity, min_angle, max_angle, angle_increment, scan_time);

  sensor_msgs::PointCloud2 cloud_out;
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
  for (std::vector<sensor_msgs::PointField>::iterator f = cloud_out.fields.begin(); f != cloud_out.fields.end(); f++)
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
        if ((max_angle - min_angle) / angle_increment > 0.0)//make sure it is not a false exception
            FAIL();
    }
  }
}


TEST(laser_geometry, transformLaserScanToPointCloud)
{

  tf::Transformer tf;
  
  double tolerance = 1e-12;
  laser_geometry::LaserProjection projector;  

  double min_angle = -M_PI/2;
  double max_angle = M_PI/2;
  double angle_increment = M_PI/180;

  double range = 1.0;
  double intensity = 1.0;

  ros::Duration scan_time = ros::Duration(1/40);
  ros::Duration increment_time = ros::Duration(1/400);


  std::vector<double> ranges, intensities, min_angles, max_angles, angle_increments;
  std::vector<ros::Duration> increment_times, scan_times;

  rostest::Permuter permuter;

  ranges.push_back(-1.0);
  ranges.push_back(1.0);
  ranges.push_back(2.0);
  ranges.push_back(3.0);
  ranges.push_back(4.0);
  ranges.push_back(5.0);
  ranges.push_back(100.0);
  permuter.addOptionSet(ranges, &range);

  intensities.push_back(1.0);
  intensities.push_back(2.0);
  intensities.push_back(3.0);
  intensities.push_back(4.0);
  intensities.push_back(5.0);
  permuter.addOptionSet(intensities, &intensity);

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

  permuter.addOptionSet(min_angles, &min_angle);
  permuter.addOptionSet(max_angles, &max_angle);

  angle_increments.push_back(-M_PI/180); // -one degree
  angle_increments.push_back(M_PI/180); // one degree
  angle_increments.push_back(M_PI/360); // half degree
  angle_increments.push_back(M_PI/720); // quarter degree
  permuter.addOptionSet(angle_increments, &angle_increment);

  scan_times.push_back(ros::Duration(1/40));
  scan_times.push_back(ros::Duration(1/20));

  permuter.addOptionSet(scan_times, &scan_time);

  while (permuter.step())
  {
    try
    {    
    //printf("%f %f %f %f %f %f\n", range, intensity, min_angle, max_angle, angle_increment, scan_time.toSec());
  sensor_msgs::LaserScan scan = build_constant_scan(range, intensity, min_angle, max_angle, angle_increment, scan_time);
  scan.header.frame_id = "laser_frame";

  sensor_msgs::PointCloud cloud_out;
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf, laser_geometry::channel_option::Index);
  EXPECT_EQ(cloud_out.channels.size(), (unsigned int)1);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf, laser_geometry::channel_option::Intensity);
  EXPECT_EQ(cloud_out.channels.size(), (unsigned int)1);

  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf);
  EXPECT_EQ(cloud_out.channels.size(), (unsigned int)2);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index);
  EXPECT_EQ(cloud_out.channels.size(), (unsigned int)2);

  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance);
  EXPECT_EQ(cloud_out.channels.size(), (unsigned int)3);

  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance | laser_geometry::channel_option::Timestamp);
  EXPECT_EQ(cloud_out.channels.size(), (unsigned int)4);

  unsigned int valid_points = 0;
  for (unsigned int i = 0; i < scan.ranges.size(); i++)
    if (scan.ranges[i] <= PROJECTION_TEST_RANGE_MAX && scan.ranges[i] >= PROJECTION_TEST_RANGE_MIN)
      valid_points ++;    
  EXPECT_EQ(valid_points, cloud_out.points.size());

  for (unsigned int i = 0; i < cloud_out.points.size(); i++)
  {
    EXPECT_NEAR(cloud_out.points[i].x , (float)((double)(scan.ranges[i]) * cos((double)(scan.angle_min) + i * (double)(scan.angle_increment))), tolerance);
    EXPECT_NEAR(cloud_out.points[i].y , (float)((double)(scan.ranges[i]) * sin((double)(scan.angle_min) + i * (double)(scan.angle_increment))), tolerance);
    EXPECT_NEAR(cloud_out.points[i].z, 0, tolerance);
    EXPECT_NEAR(cloud_out.channels[0].values[i], scan.intensities[i], tolerance);//intensity \todo determine this by lookup not hard coded order
    EXPECT_NEAR(cloud_out.channels[1].values[i], i, tolerance);//index
    EXPECT_NEAR(cloud_out.channels[2].values[i], scan.ranges[i], tolerance);//ranges
    EXPECT_NEAR(cloud_out.channels[3].values[i], (float)i * scan.time_increment, tolerance);//timestamps
  };
    }
    catch (BuildScanException &ex)
    {
        if ((max_angle - min_angle) / angle_increment > 0.0)//make sure it is not a false exception
            FAIL();
    }
  }
}

TEST(laser_geometry, transformLaserScanToPointCloud2)
{

  tf::Transformer tf;
  tf2::BufferCore tf2;

  double tolerance = 1e-12;
  laser_geometry::LaserProjection projector;  

  double min_angle = -M_PI/2;
  double max_angle = M_PI/2;
  double angle_increment = M_PI/180;

  double range = 1.0;
  double intensity = 1.0;

  ros::Duration scan_time = ros::Duration(1/40);
  ros::Duration increment_time = ros::Duration(1/400);

  std::vector<double> ranges, intensities, min_angles, max_angles, angle_increments;
  std::vector<ros::Duration> increment_times, scan_times;

  rostest::Permuter permuter;

  ranges.push_back(-1.0);
  ranges.push_back(1.0);
  ranges.push_back(2.0);
  ranges.push_back(3.0);
  ranges.push_back(4.0);
  ranges.push_back(5.0);
  ranges.push_back(100.0);
  permuter.addOptionSet(ranges, &range);

  intensities.push_back(1.0);
  intensities.push_back(2.0);
  intensities.push_back(3.0);
  intensities.push_back(4.0);
  intensities.push_back(5.0);
  permuter.addOptionSet(intensities, &intensity);

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

  permuter.addOptionSet(min_angles, &min_angle);
  permuter.addOptionSet(max_angles, &max_angle);

  angle_increments.push_back(-M_PI/180); // -one degree
  angle_increments.push_back(M_PI/180); // one degree
  angle_increments.push_back(M_PI/360); // half degree
  angle_increments.push_back(M_PI/720); // quarter degree
  permuter.addOptionSet(angle_increments, &angle_increment);

  scan_times.push_back(ros::Duration(1/40));
  scan_times.push_back(ros::Duration(1/20));

  permuter.addOptionSet(scan_times, &scan_time);

  while (permuter.step())
  {
    try
    {    
    //printf("%f %f %f %f %f %f\n", range, intensity, min_angle, max_angle, angle_increment, scan_time.toSec());
  sensor_msgs::LaserScan scan = build_constant_scan(range, intensity, min_angle, max_angle, angle_increment, scan_time);
  scan.header.frame_id = "laser_frame";

  sensor_msgs::PointCloud2 cloud_out;
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf, -1.0, laser_geometry::channel_option::None);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)3);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2, -1.0, laser_geometry::channel_option::None);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)3);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf, -1.0, laser_geometry::channel_option::Index);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)4);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2, -1.0, laser_geometry::channel_option::Index);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)4);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf, -1.0, laser_geometry::channel_option::Intensity);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)4);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2, -1.0, laser_geometry::channel_option::Intensity);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)4);

  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)5);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)5);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)5);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)5);

  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)6);
  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf2, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)6);

  projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_out, tf, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance | laser_geometry::channel_option::Timestamp);
  EXPECT_EQ(cloud_out.fields.size(), (unsigned int)7);
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
  for (std::vector<sensor_msgs::PointField>::iterator f = cloud_out.fields.begin(); f != cloud_out.fields.end(); f++)
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
        if ((max_angle - min_angle) / angle_increment > 0.0)//make sure it is not a false exception
            FAIL();
    }
  }

}


int main(int argc, char **argv){
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
