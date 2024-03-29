cmake_minimum_required(VERSION 3.0.2)
project(rl_sar)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

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

catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
)

include_directories(library/unitree_legged_sdk_3.2/include)
link_directories(library/unitree_legged_sdk_3.2/lib)
set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${unitree_legged_sdk_INCLUDE_DIRS}
    ../unitree_controller/include
    library/matplotlibcpp
)

add_library(rl library/rl/rl.cpp library/rl/rl.hpp)
target_link_libraries(rl "${TORCH_LIBRARIES}" Python3::Python Python3::Module)
set_property(TARGET rl PROPERTY CXX_STANDARD 14)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
   target_link_libraries(rl Python3::NumPy)
else()
   target_compile_definitions(rl WITHOUT_NUMPY)
endif()

add_executable(rl_sim src/rl_sim.cpp)
target_link_libraries(rl_sim
  ${catkin_LIBRARIES} ${EXTRA_LIBS} "${TORCH_LIBRARIES}"
  rl
)

add_executable(rl_real src/rl_real.cpp)
target_link_libraries(rl_real
  ${catkin_LIBRARIES} ${EXTRA_LIBS} "${TORCH_LIBRARIES}"
  rl
)