cmake_minimum_required(VERSION 3.5)
project(a1_description)

if($ENV{ROS_DISTRO} MATCHES "noetic")
	find_package(catkin REQUIRED)

	catkin_package()

	find_package(roslaunch)

	foreach(dir config launch meshes urdf)
		install(DIRECTORY ${dir}/
			DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
	endforeach(dir)
elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble|jazzy")
    find_package(ament_cmake REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    find_package(urdf REQUIRED)
    find_package(xacro REQUIRED)
    find_package(robot_state_publisher REQUIRED)
    install(
    DIRECTORY
        config
        launch
        meshes
        urdf
        xacro
    DESTINATION
        share/${PROJECT_NAME}/
    )
    ament_package()
endif()
