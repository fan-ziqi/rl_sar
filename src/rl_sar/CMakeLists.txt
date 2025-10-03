cmake_minimum_required(VERSION 3.5)
project(rl_sar VERSION 4.0.0 LANGUAGES CXX)

# Set C++ standard globally
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# System environment detection
set(SYSTEM_TYPE ${CMAKE_SYSTEM_NAME})

set(USE_CMAKE OFF CACHE BOOL "Use Cmake build system")
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

# Model Interface
add_subdirectory(library/thirdparty/model_interface)

# Find LibTorch if it was configured in the subdirectory
if(USE_TORCH)
    find_package(Torch QUIET)
endif()

find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Add Python library path to RPATH
if(Python3_LIBRARIES)
    get_filename_component(PYTHON_LIB_DIR "${Python3_LIBRARIES}" DIRECTORY)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${PYTHON_LIB_DIR}")
    message(STATUS "Added Python library path to RPATH: ${PYTHON_LIB_DIR}")
endif()

if(SYSTEM_TYPE STREQUAL "Linux")
    set(YAML_CPP_LIBRARIES yaml-cpp)
link_directories(/usr/local/lib)
include_directories(${YAML_CPP_INCLUDE_DIR})
elseif(SYSTEM_TYPE STREQUAL "Darwin")
    find_library(YAML_CPP_LIB yaml-cpp REQUIRED)
    set(YAML_CPP_LIBRARIES ${YAML_CPP_LIB})
    link_directories(/opt/homebrew/lib)
    if(YAML_CPP_INCLUDE_DIR)
        include_directories(${YAML_CPP_INCLUDE_DIR})
    endif()
endif()

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

# unitree_legged_sdk
if(SYSTEM_TYPE STREQUAL "Linux")
add_library(unitree_legged_sdk SHARED IMPORTED)
set_target_properties(unitree_legged_sdk PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_legged_sdk/include"
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_legged_sdk/lib/${UNITREE_LIB}.so"
)
set(UNITREE_A1_LIBS Threads::Threads unitree_legged_sdk lcm)
if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
    file(GLOB GLOB_UNITREE_LEGGED_SDK "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_legged_sdk/lib/${UNITREE_LIB}.so")
    install(FILES
        ${GLOB_UNITREE_LEGGED_SDK}
        DESTINATION lib/
    )
    endif()
endif()

# unitree_sdk2
if(SYSTEM_TYPE STREQUAL "Linux")
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable examples build for unitree_sdk2" FORCE)
add_subdirectory(library/thirdparty/robot_sdk/unitree/unitree_sdk2)
endif()

# l4w4_sdk
add_library(l4w4_sdk INTERFACE)
target_include_directories(l4w4_sdk INTERFACE
    library/thirdparty/robot_sdk/zhinao/l4w4_sdk
)
target_link_libraries(l4w4_sdk INTERFACE Threads::Threads)

# Lite3_MotionSDK
if(SYSTEM_TYPE STREQUAL "Linux")
add_library(lite3_motionsdk SHARED IMPORTED)
set_target_properties(lite3_motionsdk PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/Lite3_MotionSDK/include;${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/Lite3_MotionSDK/include/common"
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_${ARCH_DIR}.so"
)
set(LITE3_REAL_LIBS Threads::Threads lite3_motionsdk)
if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
    file(GLOB GLOB_LITE3_SDK_SO "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_${ARCH_DIR}.so")
    install(FILES
        ${GLOB_LITE3_SDK_SO}
        DESTINATION lib/
    )
    endif()
endif()
# Retroid Gamepad
if(SYSTEM_TYPE STREQUAL "Linux")
set(GAMEPAD_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/gamepad/src/gamepad.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/gamepad/src/retroid_gamepad.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/gamepad/src/udp_receiver.cpp
)
endif()

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
    library/core/vector_math
    library/core/model_interface
    policy
)

# library for model interface
add_library(model_interface library/core/model_interface/model_interface.cpp)
if(USE_TORCH)
    target_link_libraries(model_interface PUBLIC ${TORCH_LIBRARIES})
    target_compile_definitions(model_interface PRIVATE USE_TORCH)
endif()
if(USE_ONNX)
    target_link_libraries(model_interface PUBLIC ${ONNX_RUNTIME_LIB})
    target_compile_definitions(model_interface PRIVATE USE_ONNX)
endif()
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS model_interface DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for rl sdk
add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
target_link_libraries(rl_sdk PUBLIC
    model_interface
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

# library for observation buffer
add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS observation_buffer DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# executable for rl sim
if(NOT USE_CMAKE)
    add_executable(rl_sim src/rl_sim.cpp)
    target_link_libraries(rl_sim
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
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

# executable for rl real a1
if(SYSTEM_TYPE STREQUAL "Linux")
add_executable(rl_real_a1 src/rl_real_a1.cpp)
target_link_libraries(rl_real_a1
    ${UNITREE_A1_LIBS}
    rl_sdk
    observation_buffer
        ${YAML_CPP_LIBRARIES}
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
endif()

if(SYSTEM_TYPE STREQUAL "Linux")
add_executable(rl_real_lite3
    src/rl_real_lite3.cpp
    ${GAMEPAD_SRC}
)
target_link_libraries(rl_real_lite3
    ${LITE3_REAL_LIBS}
    rl_sdk
    observation_buffer
        ${YAML_CPP_LIBRARIES}
)
target_include_directories(rl_real_lite3 PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/gamepad/include
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
endif()

# executable for rl real go2
if(SYSTEM_TYPE STREQUAL "Linux")
add_executable(rl_real_go2 src/rl_real_go2.cpp)
target_link_libraries(rl_real_go2
    unitree_sdk2
    rl_sdk
    observation_buffer
        ${YAML_CPP_LIBRARIES}
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
endif()

# executable for rl real g1
if(SYSTEM_TYPE STREQUAL "Linux")
add_executable(rl_real_g1 src/rl_real_g1.cpp)
target_link_libraries(rl_real_g1
    unitree_sdk2
    rl_sdk
    observation_buffer
        ${YAML_CPP_LIBRARIES}
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
endif()

# executable for rl real l4w4
add_executable(rl_real_l4w4 src/rl_real_l4w4.cpp)
target_link_libraries(rl_real_l4w4
    l4w4_sdk
    rl_sdk
    observation_buffer
    ${YAML_CPP_LIBRARIES}
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

# executable for actuator net
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
# add_executable(test_vector_math test/test_vector_math.cpp)
# install(TARGETS test_vector_math DESTINATION lib/${PROJECT_NAME})

message(STATUS "========================================")
message(STATUS "       RL_SAR Build Configuration       ")
message(STATUS "========================================")
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "System: ${SYSTEM_TYPE}")
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "USE_CMAKE: ${USE_CMAKE}")
message(STATUS "----------------------------------------")
message(STATUS "Backend Configuration:")
message(STATUS "USE_TORCH: ${USE_TORCH}")
message(STATUS "USE_ONNX: ${USE_ONNX}")
if(USE_TORCH)
    if(Torch_VERSION)
        message(STATUS "LibTorch version: ${Torch_VERSION}")
    else()
        message(STATUS "LibTorch: Available")
    endif()
    if(IS_JETSON)
        message(STATUS "LibTorch: Jetson device detected")
        if(JETPACK_VERSION)
            message(STATUS "JetPack version: ${JETPACK_VERSION}")
        endif()
    endif()
endif()
if(USE_ONNX)
    if(ONNX_RUNTIME_VERSION)
        message(STATUS "ONNX Runtime version: ${ONNX_RUNTIME_VERSION}")
    elseif(onnxruntime_VERSION)
        message(STATUS "ONNX Runtime version: ${onnxruntime_VERSION}")
    else()
        message(STATUS "ONNX Runtime: Available")
    endif()
endif()
message(STATUS "========================================")
