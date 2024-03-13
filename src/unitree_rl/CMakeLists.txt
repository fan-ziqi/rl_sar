cmake_minimum_required(VERSION 3.0.2)
project(unitree_rl)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(Torch REQUIRED)

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

find_package(gazebo REQUIRED)

catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
)

include_directories(library/unitree_legged_sdk_3.2/include)
link_directories(library/unitree_legged_sdk_3.2/lib)
set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)
add_executable(lcm_server $ENV{UNITREE_LEGGED_SDK_PATH}/examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS} ${catkin_LIBRARIES})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${unitree_legged_sdk_INCLUDE_DIRS}
    ../unitree_controller/include

)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(model lib/model.cpp lib/model.hpp)
target_link_libraries(model "${TORCH_LIBRARIES}")
set_property(TARGET model PROPERTY CXX_STANDARD 14)

add_library(observation_buffer lib/observation_buffer.cpp lib/observation_buffer.hpp)
target_link_libraries(observation_buffer "${TORCH_LIBRARIES}")
set_property(TARGET observation_buffer PROPERTY CXX_STANDARD 14)

add_executable(unitree_rl src/unitree_rl.cpp )
target_link_libraries(unitree_rl
  ${catkin_LIBRARIES} ${EXTRA_LIBS} "${TORCH_LIBRARIES}"
  model observation_buffer
)

add_executable(unitree_rl_real src/unitree_rl_real.cpp )
target_link_libraries(unitree_rl_real
  ${catkin_LIBRARIES} ${EXTRA_LIBS} "${TORCH_LIBRARIES}"
  model observation_buffer
)