^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package velodyne_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.3 (2022-12-02)
------------------
* Add dummy collision to avoid moveit warnings
* Remove gazebo_ros dependency for velodyne_description
* Contributors: Filip Sund, Kevin Hallenbeck

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

1.0.11 (2021-02-23)
-------------------
* Add organize_cloud parameter to match velodyne_pointcloud
* Contributors: Kevin Hallenbeck

1.0.10 (2020-08-03)
-------------------
* Change PointCloud visualization type from flat squares to points in example rviz config
* Bump minimum CMake version to 3.0.2 in all CMakeLists.txt
* Fix xacro macro instantiation
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.9 (2019-03-08)
------------------

1.0.8 (2018-09-08)
------------------

1.0.7 (2018-07-03)
------------------
* Added GPU support
* Updated inertia tensors for VLP-16 and HDL-32E to realistic values
* Removed unnecessary file extraction code in cmake
* Contributors: Kevin Hallenbeck, Max Schwarz

1.0.6 (2017-10-17)
------------------
* Use robotNamespace as prefix for PointCloud2 topic frame_id by default
* Contributors: Micho Radovnikovich

1.0.5 (2017-09-05)
------------------
* Increased minimum collision range to prevent self-clipping when in motion
* Added many URDF parameters, and set example sample count to reasonable values
* Launch rviz with gazebo
* Contributors: Kevin Hallenbeck

1.0.4 (2017-04-24)
------------------
* Updated package.xml format to version 2
* Contributors: Kevin Hallenbeck

1.0.3 (2016-08-13)
------------------
* Contributors: Kevin Hallenbeck

1.0.2 (2016-02-03)
------------------
* Moved M_PI property out of macro to support multiple instances
* Materials caused problems with more than one sensors. Removed.
* Added example urdf and gazebo
* Changed to DAE meshes
* Added meshes. Added HDL-32E.
* Start from block laser
* Contributors: Kevin Hallenbeck
