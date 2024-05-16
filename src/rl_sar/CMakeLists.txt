cmake_minimum_required(VERSION 3.0.2)
project(rl_sar)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
find_package(Torch REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
find_package(gazebo REQUIRED)

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
    unitree_legged_msgs
)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# find_package(yaml-cpp REQUIRED)
link_directories(/usr/local/lib)
include_directories(${YAML_CPP_INCLUDE_DIR})

catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
)

include_directories(library/unitree_legged_sdk_3.2/include)
link_directories(library/unitree_legged_sdk_3.2/lib)
set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

add_subdirectory(library/cyberdog_motor_sdk)
include_directories(library/cyberdog_motor_sdk/include)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${unitree_legged_sdk_INCLUDE_DIRS}
    ../unitree_controller/include
    library/matplotlibcpp
    library/observation_buffer
    library/rl_sdk
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
  ${catkin_LIBRARIES} ${EXTRA_LIBS} "${TORCH_LIBRARIES}"
  rl_sdk observation_buffer yaml-cpp
)

add_executable(rl_real_a1 src/rl_real_a1.cpp )
target_link_libraries(rl_real_a1
  ${catkin_LIBRARIES} ${EXTRA_LIBS} "${TORCH_LIBRARIES}"
  rl_sdk observation_buffer yaml-cpp
)


add_executable(rl_real_cyberdog src/rl_real_cyberdog.cpp )
target_link_libraries(rl_real_cyberdog
  ${catkin_LIBRARIES} ${EXTRA_LIBS} "${TORCH_LIBRARIES}"
  rl_sdk observation_buffer cyberdog_motor_sdk yaml-cpp
)