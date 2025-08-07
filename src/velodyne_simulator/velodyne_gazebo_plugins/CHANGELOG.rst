^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package velodyne_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.3 (2022-12-02)
------------------
* Fix logger print warning
* Update cmake for best practices based on other packages
* Default to C++17
* Contributors: Kevin Hallenbeck

2.0.2 (2021-12-27)
------------------
* Adds tf_prefix SDF parameter
* Contributors: Micho Radovnikovich

2.0.1 (2021-10-14)
------------------

2.0.0 (2021-10-12)
------------------
* Merged in ROS2 Foxy support (pull request #14)
  * Remove support for old Gazebo versions
  * Lazy subscriber using timer in the absence of publisher connection callback
  * Python launch file
  * Add env-hooks for GAZEBO_MODEL_PATH
* Contributors: Kevin Hallenbeck, Gonzalo de Pedro, Joep Tool

1.0.12 (2021-03-30)
-------------------
* Fix swapped PointCloud2 width/height
  Bug introduced in recent commit adding organize_cloud option
* Contributors: Kevin Hallenbeck

1.0.11 (2021-02-23)
-------------------
* Remove support for end-of-life ROS distributions
* Add organize_cloud parameter to match velodyne_pointcloud
* Contributors: Kevin Hallenbeck

1.0.10 (2020-08-03)
-------------------
* Change PointCloud2 structure to match updated velodyne_pointcloud package
* Bump minimum CMake version to 3.0.2 in all CMakeLists.txt
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.9 (2019-03-08)
------------------
* Added min_intensity parameter to support cliping of low intensity returns
* Contributors: Jonathan Wheare, Kevin Hallenbeck

1.0.8 (2018-09-08)
------------------
* Changed iteration order to more closely represent the live velodyne driver
* Contributors: Kevin Hallenbeck

1.0.7 (2018-07-03)
------------------
* Added GPU support
* Added support for Gazebo 9
* Improved behavior of max range calculation
* Removed trailing slashes in robot namespace
* Fixed resolution of 1 not supported
* Fixed issue with only 1 vert or horiz ray
* Fixed cmake exports and warning
* Contributors: Kevin Hallenbeck, Jacob Seibert, Naoki Mizuno

1.0.6 (2017-10-17)
------------------
* Use robotNamespace as prefix for PointCloud2 topic frame_id by default
* Use Gazebo LaserScan message instead of direct LaserShape access, fixes timestamp issue
* Contributors: Kevin Hallenbeck, Max Schwarz, Micho Radovnikovich

1.0.5 (2017-09-05)
------------------
* Fixed ground plane projection by removing interpolation
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.4 (2017-04-24)
------------------
* Updated package.xml format to version 2
* Removed gazebo_plugins dependency
* Contributors: Kevin Hallenbeck

1.0.3 (2016-08-13)
------------------
* Gazebo7 integration
* Contributors: Kevin Hallenbeck, Konstantin Sorokin

1.0.2 (2016-02-03)
------------------
* Display laser count when loading gazebo plugin
* Don't reverse ring for newer gazebo versions
* Changed to PointCloud2. Handle min and max range. Noise. General cleanup.
* Start from block laser
* Contributors: Kevin Hallenbeck
