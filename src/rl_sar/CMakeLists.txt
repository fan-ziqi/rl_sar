cmake_minimum_required(VERSION 3.5)
project(rl_sar)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
find_package(Torch REQUIRED)
set(CMAKE_INSTALL_RPATH "${Torch_DIR}/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
find_package(gazebo REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(robot_joint_controller REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Make sure to find correct Python version based on ROS distro
if(DEFINED ENV{ROS_DISTRO})
    set(ROS_DISTRO_ENV $ENV{ROS_DISTRO})
    if(ROS_DISTRO_ENV STREQUAL "foxy")
      find_package(Python3 3.8 EXACT COMPONENTS Interpreter Development REQUIRED)
    elseif(ROS_DISTRO_ENV STREQUAL "humble")
      find_package(Python3 3.10 EXACT COMPONENTS Interpreter Development REQUIRED)
    else()
      # Not very sure what would the default case be. A fuzzy match?
      find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    endif()
endif()


link_directories(/usr/local/lib)
include_directories(${YAML_CPP_INCLUDE_DIR})

# Unitree A1
include_directories(library/unitree_legged_sdk_3.2/include)
link_directories(library/unitree_legged_sdk_3.2/lib)
set(UNITREE_A1_LIBS -pthread unitree_legged_sdk_amd64 lcm)
file(GLOB GLOB_UNITREE_LEGGED_SDK "${PROJECT_SOURCE_DIR}/library/unitree_legged_sdk_3.2/lib/*.so")
install(FILES
  ${GLOB_UNITREE_LEGGED_SDK}
  DESTINATION lib/
)

# Unitree Go2
include_directories(library/unitree_sdk2/include)
link_directories(library/unitree_sdk2/lib/x86_64)
include_directories(library/unitree_sdk2/thirdparty/include)
include_directories(library/unitree_sdk2/thirdparty/include/ddscxx)
link_directories(library/unitree_sdk2/thirdparty/lib/x86_64)
set(UNITREE_GO2_LIBS -pthread unitree_sdk2 ddsc ddscxx)
file(GLOB GLOB_UNITREE_SDK2 "${PROJECT_SOURCE_DIR}/library/unitree_sdk2/lib/x86_64/*.so")
file(GLOB GLOB_UNITREE_SDK2_THIRDPARTY "${PROJECT_SOURCE_DIR}/library/unitree_sdk2/thirdparty/lib/x86_64/*.so")
install(FILES
  ${GLOB_UNITREE_SDK2_THIRDPARTY}
  ${GLOB_UNITREE_SDK2_THIRDPARTY}
  DESTINATION lib/
)

include_directories(
  include
  library/matplotlibcpp
  library/observation_buffer
  library/rl_sdk
  library/loop
)

add_library(rl_sdk library/rl_sdk/rl_sdk.cpp)
target_link_libraries(rl_sdk "${TORCH_LIBRARIES}" Python3::Python Python3::Module)
set_property(TARGET rl_sdk PROPERTY CXX_STANDARD 14)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(rl_sdk Python3::NumPy)
else()
  target_compile_definitions(rl_sdk WITHOUT_NUMPY)
endif()

add_library(observation_buffer library/observation_buffer/observation_buffer.cpp)
target_link_libraries(observation_buffer "${TORCH_LIBRARIES}")
set_property(TARGET observation_buffer PROPERTY CXX_STANDARD 14)

add_executable(rl_sim src/rl_sim.cpp )
target_link_libraries(rl_sim
  -pthread
  rl_sdk observation_buffer yaml-cpp
)

add_executable(rl_real_a1 src/rl_real_a1.cpp )
target_link_libraries(rl_real_a1
  ${UNITREE_A1_LIBS}
  rl_sdk observation_buffer yaml-cpp
)

add_executable(rl_real_go2 src/rl_real_go2.cpp )
target_link_libraries(rl_real_go2
  ${UNITREE_GO2_LIBS}
  rl_sdk observation_buffer yaml-cpp
)

ament_target_dependencies(rl_sim
  joint_state_broadcaster
  robot_state_publisher
  rclcpp
  gazebo_ros
  std_msgs
  robot_msgs
  robot_joint_controller
  rclpy
  gazebo_msgs
  std_srvs
)

install(TARGETS
  rl_sim
  rl_sdk
  observation_buffer
  rl_real_a1
  rl_real_go2
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/rl_sim.py
  scripts/actuator_net.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
