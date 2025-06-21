cmake_minimum_required(VERSION 3.5)
project(a1_description)

if($ENV{ROS_DISTRO} MATCHES "noetic")
    find_package(catkin REQUIRED COMPONENTS
        genmsg
        roscpp
        std_msgs
        tf
    )
    catkin_package(
        CATKIN_DEPENDS
    )
    include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
    )
elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble|jazzy")
    find_package(ament_cmake REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    find_package(urdf REQUIRED)
    find_package(xacro REQUIRED)
    find_package(robot_state_publisher REQUIRED)
    install(
    DIRECTORY
        config_ros2
        launch
        meshes
        urdf
        xacro
    DESTINATION
        share/${PROJECT_NAME}/
    )
    ament_package()
endif()
