cmake_minimum_required(VERSION 3.5)
project(lite3_description VERSION 3.0.0)

if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
    find_package(ament_cmake REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    find_package(urdf REQUIRED)
    find_package(xacro REQUIRED)
    find_package(robot_state_publisher REQUIRED)
    foreach(dir config launch meshes urdf xacro)
        if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
            install(
                DIRECTORY ${dir}/
                DESTINATION share/${PROJECT_NAME}/${dir}
            )
        endif()
    endforeach()
    ament_package()
endif()

