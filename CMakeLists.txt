cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rospack(laser_geometry)

rospack_add_boost_directories()

rospack_add_library(laser_geometry src/laser_geometry.cpp )
rospack_link_boost(laser_geometry thread)

rospack_add_gtest(test/projection_test test/projection_test.cpp)
target_link_libraries (test/projection_test laser_geometry)

