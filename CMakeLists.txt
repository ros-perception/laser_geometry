cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

rosbuild_add_boost_directories()

rosbuild_add_library(laser_geometry src/laser_geometry.cpp )
rosbuild_link_boost(laser_geometry thread)
rosbuild_add_library (laser_scan_geometry src/laser_scan_geometry.cpp)

rosbuild_add_gtest(test/projection_test test/projection_test.cpp)
target_link_libraries (test/projection_test laser_geometry)

