cmake_minimum_required(VERSION 3.5)
project(rl_sar VERSION 3.0.0 LANGUAGES CXX)

set(USE_CMAKE OFF CACHE BOOL "Use Cmake build system")
message(STATUS "USE_CMAKE: ${USE_CMAKE}")
if(USE_CMAKE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        add_compile_definitions(USE_ROS1)
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        add_compile_definitions(USE_ROS2)
    endif()
else()
    message(WARNING "ROS_DISTRO not set, assuming non-ROS mode")
    add_compile_definitions(USE_CMAKE)
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
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
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
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
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
        find_package(geometry_msgs REQUIRED)
        ament_package()
    endif()
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
if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
    file(GLOB GLOB_UNITREE_LEGGED_SDK "${CMAKE_CURRENT_SOURCE_DIR}/library/unitree_legged_sdk-3.2/lib/${UNITREE_LIB}.so")
    install(FILES
        ${GLOB_UNITREE_LEGGED_SDK}
        DESTINATION lib/
    )
endif()

# unitree_sdk2-2.0.0
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable examples build for unitree_sdk2" FORCE)
add_subdirectory(library/thirdparty/unitree_sdk2-2.0.0)

# l4w4_sdk
add_library(l4w4_sdk INTERFACE)
target_include_directories(l4w4_sdk INTERFACE
    library/thirdparty/l4w4_sdk
)
target_link_libraries(l4w4_sdk INTERFACE Threads::Threads)

# Lite3_MotionSDK
add_library(lite3_motionsdk SHARED IMPORTED)
set_target_properties(lite3_motionsdk PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/Lite3_MotionSDK/include"
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_${ARCH_DIR}.so"
)
set(LITE3_REAL_LIBS Threads::Threads lite3_motionsdk)
if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
    file(GLOB GLOB_LITE3_SDK_SO "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_${ARCH_DIR}.so")
    install(FILES
        ${GLOB_LITE3_SDK_SO}
        DESTINATION lib/
    )
endif()

# Retroid Gamepad
set(GAMEPAD_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/gamepad/src/gamepad.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/gamepad/src/retroid_gamepad.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/gamepad/src/udp_receiver.cpp
)

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        include_directories(${catkin_INCLUDE_DIRS})
    endif()
endif()

include_directories(
    include
    library/core/matplotlibcpp
    library/core/observation_buffer
    library/core/rl_sdk
    library/core/loop
    library/core/fsm
    policy
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
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS rl_sdk DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
target_link_libraries(observation_buffer PUBLIC "${TORCH_LIBRARIES}")
set_target_properties(observation_buffer PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS observation_buffer DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

if(NOT USE_CMAKE)
    add_executable(rl_sim src/rl_sim.cpp)
    target_link_libraries(rl_sim
        rl_sdk
        observation_buffer
        yaml-cpp
        Threads::Threads
    )
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        target_link_libraries(rl_sim ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
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
            geometry_msgs
        )
        install(TARGETS rl_sim DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

add_executable(rl_real_a1 src/rl_real_a1.cpp)
target_link_libraries(rl_real_a1
    ${UNITREE_A1_LIBS}
    rl_sdk
    observation_buffer
    yaml-cpp
)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        target_link_libraries(rl_real_a1 ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        ament_target_dependencies(rl_real_a1
            rclcpp
            geometry_msgs
        )
        install(TARGETS rl_real_a1 DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

add_executable(rl_real_lite3
    src/rl_real_lite3.cpp
    ${GAMEPAD_SRC}
)
target_link_libraries(rl_real_lite3
    ${LITE3_REAL_LIBS}
    rl_sdk
    observation_buffer
    yaml-cpp
)
target_include_directories(rl_real_lite3 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/gamepad/include
)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        target_link_libraries(rl_real_lite3 ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        ament_target_dependencies(rl_real_lite3
            rclcpp
            geometry_msgs
        )
        install(TARGETS rl_real_lite3 DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

add_executable(rl_real_go2 src/rl_real_go2.cpp)
target_link_libraries(rl_real_go2
    unitree_sdk2
    rl_sdk
    observation_buffer
    yaml-cpp
)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        target_link_libraries(rl_real_go2 ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        ament_target_dependencies(rl_real_go2
            rclcpp
            geometry_msgs
        )
        install(TARGETS rl_real_go2 DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

add_executable(rl_real_g1 src/rl_real_g1.cpp)
target_link_libraries(rl_real_g1
    unitree_sdk2
    rl_sdk
    observation_buffer
    yaml-cpp
)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        target_link_libraries(rl_real_g1 ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        ament_target_dependencies(rl_real_g1
            rclcpp
            geometry_msgs
        )
        install(TARGETS rl_real_g1 DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

add_executable(rl_real_l4w4 src/rl_real_l4w4.cpp)
target_link_libraries(rl_real_l4w4
    l4w4_sdk
    rl_sdk
    observation_buffer
    yaml-cpp
)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        target_link_libraries(rl_real_l4w4 ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        ament_target_dependencies(rl_real_l4w4
            rclcpp
            geometry_msgs
        )
        install(TARGETS rl_real_l4w4 DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        catkin_install_python(PROGRAMS
            scripts/actuator_net.py
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(PROGRAMS
            scripts/actuator_net.py
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(
            DIRECTORY launch worlds policy
            DESTINATION share/${PROJECT_NAME}/${dir}
        )
    endif()
endif()

# only for test
# add_executable(test_observation_buffer test/test_observation_buffer.cpp)
# target_link_libraries(test_observation_buffer
#     observation_buffer
# )
# install(TARGETS test_observation_buffer DESTINATION lib/${PROJECT_NAME})
