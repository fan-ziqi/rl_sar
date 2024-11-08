cmake_minimum_required(VERSION 3.8)
project(go2_description)

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
