cmake_minimum_required(VERSION 2.4.6)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

rosbuild_add_boost_directories()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

rosbuild_add_library(laser_geometry src/laser_geometry.cpp )
rosbuild_link_boost(laser_geometry thread)

rosbuild_add_gtest(test/projection_test test/projection_test.cpp)
target_link_libraries (test/projection_test laser_geometry)

