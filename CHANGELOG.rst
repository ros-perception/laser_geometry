^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laser_geometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
