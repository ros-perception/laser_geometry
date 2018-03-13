cmake_minimum_required(VERSION 3.5)

project(laser_geometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

# TODO(dhood): enable python support once ported to ROS 2
# catkin_python_setup()

include_directories(include
    ${angles_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(laser_geometry src/laser_geometry.cpp)
target_link_libraries(laser_geometry
    ${angles_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${tf2_LIBRARIES}
)

set_property(TARGET laser_geometry PROPERTY POSITION_INDEPENDENT_CODE ON)

ament_export_include_directories(include)
ament_export_interfaces(laser_geometry)
ament_export_libraries(laser_geometry)
ament_package(CONFIG_EXTRAS laser_geometry-extras.cmake)

install(
  TARGETS laser_geometry
  EXPORT laser_geometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

