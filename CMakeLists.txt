cmake_minimum_required(VERSION 2.8.3)
project(laser_geometry)
find_package(catkin REQUIRED sensor_msgs roscpp tf angles)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_geometry
  DEPENDS Boost Eigen
)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(laser_geometry src/laser_geometry.cpp)
target_link_libraries(laser_geometry ${Boost_LIBRARIES} ${tf_LIBRARIES})

catkin_add_gtest(projection_test test/projection_test.cpp)
target_link_libraries(projection_test laser_geometry)

install(TARGETS laser_geometry
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/laser_geometry/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
