cmake_minimum_required(VERSION 3.5.0)
project(velodyne_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz urdf world
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/velodyne_description.dsv.in")

ament_package()