cmake_minimum_required(VERSION 2.8.3)
project(laser_geometry)
find_package(catkin REQUIRED sensor_msgs roscpp tf angles)
catkin_package(
  LIBRARIES laser_geometry
  INCLUDE_DIRS include
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Boost 1.40.0 COMPONENTS thread numeric)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

include_directories(include)

add_library(laser_geometry src/laser_geometry.cpp )
target_link_libraries(laser_geometry ${Boost_LIBRARIES} ${tf_LIBRARIES})

catkin_add_gtest(projection_test test/projection_test.cpp)
target_link_libraries(projection_test laser_geometry)
