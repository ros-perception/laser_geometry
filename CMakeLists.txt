cmake_minimum_required(VERSION 3.5)

project(laser_geometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_python_install_package(laser_geometry PACKAGE_DIR src/laser_geometry)

add_library(laser_geometry SHARED src/laser_geometry.cpp)
target_include_directories(laser_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(laser_geometry PUBLIC
  ${sensor_msgs_TARGETS}
  tf2::tf2
)
if(TARGET Eigen3::Eigen)
  target_link_libraries(laser_geometry PUBLIC Eigen3::Eigen)
else()
  target_include_directories(laser_geometry PUBLIC ${Eigen3_INCLUDE_DIRS})
endif()

target_link_libraries(laser_geometry PRIVATE
  rclcpp::rclcpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(laser_geometry PRIVATE "LASER_GEOMETRY_BUILDING_LIBRARY")

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(laser_geometry)

# Export modern CMake targets
ament_export_targets(laser_geometry)

ament_export_dependencies(
  Eigen3
  sensor_msgs
  tf2
)

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
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # TODO(Martin-Idel-SI): replace this with ament_lint_auto() and/or add the copyright linter back
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cppcheck()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(projection_test
    test/projection_test.cpp
    TIMEOUT 240)
  if(TARGET projection_test)
    target_link_libraries(projection_test laser_geometry rclcpp::rclcpp)
  endif()

  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(projection test/projection_test.py
    TIMEOUT 120
  )
endif()

ament_package()
