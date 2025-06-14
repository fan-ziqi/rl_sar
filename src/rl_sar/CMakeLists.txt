cmake_minimum_required(VERSION 3.5)
project(rl_sar)

set(USE_CATKIN ON CACHE BOOL "Use Catkin build system")
message(STATUS "USE_CATKIN: ${USE_CATKIN}")
if(NOT USE_CATKIN)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
  add_compile_options(-g)
endif()

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

if(USE_CATKIN)
  find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    robot_msgs
    robot_joint_controller
    rospy
  )

  catkin_package(
    CATKIN_DEPENDS
    robot_joint_controller
    rospy
  )

  add_compile_options(${GAZEBO_CXX_FLAGS})
  find_package(gazebo REQUIRED)
endif()

find_package(Torch REQUIRED)
find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

link_directories(/usr/local/lib)
include_directories(${YAML_CPP_INCLUDE_DIR})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    set(ARCH_DIR "aarch64")
    set(UNITREE_LIB "libunitree_legged_sdk_arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    set(ARCH_DIR "x86_64")
    set(UNITREE_LIB "libunitree_legged_sdk_amd64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# unitree_legged_sdk-3.2
add_library(unitree_legged_sdk SHARED IMPORTED)
set_target_properties(unitree_legged_sdk PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/unitree_legged_sdk-3.2/include"
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/unitree_legged_sdk-3.2/lib/${UNITREE_LIB}.so"
)
set(UNITREE_A1_LIBS Threads::Threads unitree_legged_sdk lcm)

# unitree_sdk2-2.0.0
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable examples build for unitree_sdk2" FORCE)
add_subdirectory(library/thirdparty/unitree_sdk2-2.0.0)

# l4w4_sdk
add_library(l4w4_sdk INTERFACE)
target_include_directories(l4w4_sdk INTERFACE
    library/thirdparty/l4w4_sdk
)
target_link_libraries(l4w4_sdk INTERFACE Threads::Threads)

if(USE_CATKIN)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

include_directories(
  include
  library/core/matplotlibcpp
  library/core/observation_buffer
  library/core/rl_sdk
  library/core/loop
  library/core/fsm
)

add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
set_target_properties(rl_sdk PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)
target_link_libraries(rl_sdk PUBLIC
  "${TORCH_LIBRARIES}"
  Python3::Python
  Python3::Module
  TBB::tbb
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(rl_sdk PUBLIC Python3::NumPy)
else()
  target_compile_definitions(rl_sdk PUBLIC WITHOUT_NUMPY)
endif()

add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
target_link_libraries(observation_buffer PUBLIC "${TORCH_LIBRARIES}")
set_target_properties(observation_buffer PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

if(USE_CATKIN)
  add_executable(rl_sim src/rl_sim.cpp)
  target_link_libraries(rl_sim PRIVATE
    rl_sdk
    observation_buffer
    yaml-cpp
    Threads::Threads
    ${catkin_LIBRARIES}
  )
endif()

add_executable(rl_real_a1 src/rl_real_a1.cpp)
target_link_libraries(rl_real_a1 PRIVATE
  ${UNITREE_A1_LIBS}
  rl_sdk
  observation_buffer
  yaml-cpp
)
if(USE_CATKIN)
  target_link_libraries(rl_real_a1 PRIVATE ${catkin_LIBRARIES})
endif()

add_executable(rl_real_go2 src/rl_real_go2.cpp)
target_link_libraries(rl_real_go2 PRIVATE
  unitree_sdk2
  rl_sdk
  observation_buffer
  yaml-cpp
)
if(USE_CATKIN)
  target_link_libraries(rl_real_go2 PRIVATE ${catkin_LIBRARIES})
endif()

add_executable(rl_real_l4w4 src/rl_real_l4w4.cpp)
target_link_libraries(rl_real_l4w4 PRIVATE
  l4w4_sdk
  rl_sdk
  observation_buffer
  yaml-cpp
)
if(USE_CATKIN)
  target_link_libraries(rl_real_l4w4 PRIVATE ${catkin_LIBRARIES})
endif()

if(USE_CATKIN)
  catkin_install_python(PROGRAMS
    scripts/rl_sim.py
    scripts/actuator_net.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()
