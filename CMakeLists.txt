cmake_minimum_required(VERSION 3.0.2)
project(laser_geometry)

find_package(catkin REQUIRED
    COMPONENTS
        angles
        roscpp
        sensor_msgs
        tf
        tf2
)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_geometry
  CATKIN_DEPENDS tf tf2
  DEPENDS Boost EIGEN3
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(laser_geometry src/laser_geometry.cpp)
target_link_libraries(laser_geometry ${Boost_LIBRARIES} ${tf_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(projection_test test/projection_test.cpp)
  target_include_directories(projection_test PRIVATE ${rostest_INCLUDE_DIRS})
  target_link_libraries(projection_test laser_geometry)

  catkin_add_nosetests(test/projection_test.py)
endif()

install(TARGETS laser_geometry
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/laser_geometry/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
