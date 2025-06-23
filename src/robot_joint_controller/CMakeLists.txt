cmake_minimum_required(VERSION 3.5)
project(robot_joint_controller VERSION 3.0.0 LANGUAGES CXX)

if($ENV{ROS_DISTRO} MATCHES "noetic")
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
        INCLUDE_DIRS ros/include
        LIBRARIES ${PROJECT_NAME}
    )
    include_directories(ros/include ${catkin_INCLUDE_DIRS})
    link_directories(${catkin_LIB_DIRS} lib)
    add_library(robot_joint_controller
        ros/src/robot_joint_controller.cpp
    )
    target_link_libraries(robot_joint_controller ${catkin_LIBRARIES})
elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        add_definitions(-DROS_DISTRO_FOXY)
    elseif($ENV{ROS_DISTRO} MATCHES "humble")
        add_definitions(-DROS_DISTRO_HUMBLE)
    else()
        add_definitions(-DROS_DISTRO_UNKNOWN)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(urdf REQUIRED)
    find_package(control_toolbox REQUIRED)
    find_package(realtime_tools REQUIRED)
    find_package(hardware_interface REQUIRED)
    find_package(controller_interface REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(robot_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(rcl_interfaces REQUIRED)

    include_directories(ros2/include)

    add_library(${PROJECT_NAME} SHARED
        ros2/src/robot_joint_controller.cpp
        ros2/src/robot_joint_controller_group.cpp
    )

    target_include_directories(${PROJECT_NAME} PRIVATE ros2/include)

    ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        urdf
        control_toolbox
        realtime_tools
        hardware_interface
        controller_interface
        pluginlib
        robot_msgs
        geometry_msgs
        rcl_interfaces
    )

    pluginlib_export_plugin_description_file(controller_interface ros2/robot_joint_controller_plugins.xml)

    install(DIRECTORY ros2/include/
        DESTINATION include/${PROJECT_NAME}
    )

    ament_export_libraries(${PROJECT_NAME})
    ament_export_dependencies(
        rclcpp
        urdf
        control_toolbox
        realtime_tools
        hardware_interface
        controller_interface
        pluginlib
        robot_msgs
        geometry_msgs
        rcl_interfaces
    )
    install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
    )

    ament_package()
endif()
