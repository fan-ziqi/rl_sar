cmake_minimum_required(VERSION 2.8.3)
project(robot_joint_controller)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    realtime_tools
    robot_msgs
)

catkin_package(
    CATKIN_DEPENDS
    robot_msgs
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIB_DIRS} lib)

add_library(robot_joint_controller 
    src/robot_joint_controller.cpp
)
target_link_libraries(robot_joint_controller ${catkin_LIBRARIES})
