# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# copied from rclcpp/rclcpp-extras.cmake

# register ament_package() hook for node plugins once

find_package(Eigen3 REQUIRED)

# Eigen3 uses non-standard variable for
# include dirs (case and name): EIGEN3_INCLUDE_DIR.
if(NOT Eigen3_INCLUDE_DIRS)
  if (EIGEN3_INCLUDE_DIR)
    message(STATUS "append ${EIGEN3_INCLUDE_DIR} to (${laser_geometry_INCLUDE_DIRS})")
	list(APPEND laser_geometry_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
    message(STATUS "laser_geometry_INCLUDE_DIRS=${laser_geometry_INCLUDE_DIRS}")
  else()
    message(FATAL_ERROR "Eigen3_INCLUDE_DIRS not found")
  endif()
endif()
