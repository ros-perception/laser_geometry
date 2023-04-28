^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laser_geometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.0 (2023-04-28)
------------------

2.5.0 (2023-02-14)
------------------
* Update laser_geometry to C++17. (`#90 <https://github.com/ros-perception/laser_geometry/issues/90>`_)
* Update Maintainers (`#88 <https://github.com/ros-perception/laser_geometry/issues/88>`_)
* Mirror rolling to ros2
* Contributors: Audrow Nash, Chris Lalancette

2.4.0 (2022-03-01)
------------------
* Install headers to include/${PROJECT_NAME} (`#86 <https://github.com/ros-perception/laser_geometry/issues/86>`_)
* Explicit cast to double to prevent loss of precision
* Fix Duration casting issue leading to no undistortion
* Contributors: Jonathan Binney, Marco Lampacrescia, Shane Loretz

2.3.0 (2022-01-14)
------------------
* Fix building on running on Windows Debug (`#82 <https://github.com/ros-perception/laser_geometry/issues/82>`_)
* Update python code and tests for ros2 (`#80 <https://github.com/ros-perception/laser_geometry/issues/80>`_)
* Contributors: Chris Lalancette, Jonathan Binney

2.2.2 (2021-05-11)
------------------
* Export sensor_msgs, tf2, and rclcpp as dependencies
* Contributors: Mabel Zhang, Michel Hidalgo

2.2.1 (2020-12-08)
------------------
* Use rclcpp::Duration::from_seconds (`#72 <https://github.com/ros-perception/laser_geometry/issues/72>`_)
* update maintainers
* increase test timeout
* Contributors: Dirk Thomas, Ivan Santiago Paunovic, Jonathan Binney, Mabel Zhang

2.2.0 (2020-04-30)
------------------
* use ament_export_targets()
* code style only: wrap after open parenthesis if not in one line (`#52 <https://github.com/ros-perception/laser_geometry/issues/52>`_)
* use target_include_directories
* Drop CMake extras redundant with eigen3_cmake_module. (`#50 <https://github.com/ros-perception/laser_geometry/issues/50>`_)
* Contributors: Dirk Thomas, Jonathan Binney, Karsten Knese, Michel Hidalgo

2.1.0 (2019-09-27)
------------------
* Merge pull request `#46 <https://github.com/ros-perception/laser_geometry/issues/46>`_ from sloretz/eigen3_cmake_module
* Contributors: Jonathan Binney, Shane Loretz

2.0.0 (2018-06-27)
------------------
* Removed the ``angle`` dependency as no longer necessary.
* Updated to build statically but use position independent code.
* Updated to compile, and to remove PointCloud support, and remove boost.
* Added visibility headers modified from ``rclcpp``.
* Updated ``laser_geometry`` to build for ros2 (and on Windows 10).
* Improved use of numpy. (`#14 <https://github.com/ros-perception/laser_geometry/issues/14>`_)
* Contributors: Alessandro Bottero, Andreas Greimel, Brian Fjeldstad, Eric Wieser, Jon Binney, Jonathan Binney, Martin Idel, Mikael Arguedas, Vincent Rabaud, William Woodall

1.6.4 (2015-05-18)
------------------
* Fix segfault when laserscan ranges[] is empty
* Contributors: Timm Linder, Vincent Rabaud

1.6.3 (2015-03-07)
------------------
* provide support for tf2
* Contributors: Vincent Rabaud

1.6.2 (2014-06-08)
------------------
* adds python port (only simple projection)
* allows to have range_cutoff > range_max
  NOTE this is required if we need to keep the range_max readings
  in the point cloud.
  An example application is an obstacle_layer in a costmap.
* Contributors: Vincent Rabaud, enriquefernandez

1.6.1 (2014-02-23)
------------------
* Added dependency on cmake_modules
* Contributors: William Woodall

1.6.0 (2014-02-21)
------------------
* Adding William Woodall as a co-maintainer
* Contributors: Vincent Rabaud, William Woodall

1.5.15 (2013-12-02)
-------------------
* Fix mistake in end_time calculation for scan transformation in #6

1.5.14 (2013-11-04)
-------------------
* Treat max_range as invalid measurement
* Properly propagate range_cutoff
* check for CATKIN_ENABLE_TESTING

1.5.13 (2013-10-06)
-------------------
* fixes `#3 <https://github.com/ros-perception/laser_geometry/issues/3>`_

1.5.12 (2013-09-14)
-------------------
* fix case of Eigen find_package name

1.5.11 (2013-07-01)
-------------------
* added missing run deps

1.5.10 (2013-06-28 15:09)
-------------------------
* [bugfix] export boost and eigen via DEPENDS

1.5.9 (2013-06-28 11:38)
------------------------
* [bugfix] export boost and eigen include dirs

1.5.8 (2012-12-14 13:54)
------------------------
* Added buildtool_depend on catkin

1.5.7 (2012-12-14 13:48)
------------------------
* CMake clean up

1.5.6 (2012-12-10)
------------------
* Removed vestigial manifest.xml

1.5.5 (2012-11-15)
------------------
* Added .count field (of 1) to every PointCloud2 field description.
  This fixes the bug referred to here: http://dev.pointclouds.org/issues/821 which is useful because that fix in PCL
  seems not to be released yet.
  Also this way is more correct, as far as I can tell.
* Tidied up CMakeLists.txt based on Dirk's recommendations.

1.5.4 (2012-10-10)
------------------
* added install rules to CMakeLists.txt needed for catkinization.
* catkinized
