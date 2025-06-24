cmake_minimum_required(VERSION 3.5)
project(rl_sar)

# 添加宏定义，将当前源码目录路径定义为编译时常量
add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# 生成compile_commands.json文件（用于静态分析工具如clangd）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置构建类型为Debug模式
set(CMAKE_BUILD_TYPE Debug)

# 添加调试符号编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

##################################################
# LibTorch配置部分
##################################################
# 添加Torch的编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
# 查找Torch库（PyTorch C++接口）
find_package(Torch REQUIRED)

# 设置运行时库路径(RPATH)相关配置
set(CMAKE_INSTALL_RPATH "${Torch_DIR}/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

##################################################
# Gazebo配置部分
##################################################
# 添加Gazebo的编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# 查找Gazebo库
find_package(gazebo REQUIRED)

##################################################
# ROS2相关依赖配置
##################################################
# 查找ROS2基础构建工具
find_package(ament_cmake REQUIRED)
# 查找ROS2相关功能包
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

##################################################
# Python版本配置（根据ROS发行版选择）
##################################################
if(DEFINED ENV{ROS_DISTRO})
    set(ROS_DISTRO_ENV $ENV{ROS_DISTRO})
    # ROS2 Foxy使用Python3.8
    if(ROS_DISTRO_ENV STREQUAL "foxy")
      find_package(Python3 3.8 EXACT COMPONENTS Interpreter Development REQUIRED)
    # ROS2 Humble使用Python3.10
    elseif(ROS_DISTRO_ENV STREQUAL "humble")
      find_package(Python3 3.10 EXACT COMPONENTS Interpreter Development REQUIRED)
    else()
      # 其他情况使用默认Python版本
      find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    endif()
endif()

##################################################
# 链接目录和包含目录配置
##################################################
# 添加系统库链接目录
link_directories(/usr/local/lib)
# 添加YAML库包含目录
include_directories(${YAML_CPP_INCLUDE_DIR})

##################################################
# Unitree A1机器人SDK配置
##################################################
# 添加A1 SDK头文件路径
include_directories(library/unitree_legged_sdk_3.2/include)
# 添加A1 SDK库路径
link_directories(library/unitree_legged_sdk_3.2/lib)
# 设置A1 SDK依赖库
set(UNITREE_A1_LIBS -pthread unitree_legged_sdk_amd64 lcm)
# 收集A1 SDK的所有.so文件
file(GLOB GLOB_UNITREE_LEGGED_SDK "${PROJECT_SOURCE_DIR}/library/unitree_legged_sdk_3.2/lib/*.so")
# 安装A1 SDK库文件到系统
install(FILES
  ${GLOB_UNITREE_LEGGED_SDK}
  DESTINATION lib/
)

##################################################
# Unitree Go2机器人SDK配置
##################################################
# 添加Go2 SDK相关路径
include_directories(library/unitree_sdk2/include)
link_directories(library/unitree_sdk2/lib/x86_64)
include_directories(library/unitree_sdk2/thirdparty/include)
include_directories(library/unitree_sdk2/thirdparty/include/ddscxx)
link_directories(library/unitree_sdk2/thirdparty/lib/x86_64)
# 设置Go2 SDK依赖库
set(UNITREE_GO2_LIBS -pthread unitree_sdk2 ddsc ddscxx)
# 收集Go2 SDK的所有.so文件
file(GLOB GLOB_UNITREE_SDK2 "${PROJECT_SOURCE_DIR}/library/unitree_sdk2/lib/x86_64/*.so")
file(GLOB GLOB_UNITREE_SDK2_THIRDPARTY "${PROJECT_SOURCE_DIR}/library/unitree_sdk2/thirdparty/lib/x86_64/*.so")
# 安装Go2 SDK库文件到系统
install(FILES
  ${GLOB_UNITREE_SDK2_THIRDPARTY}
  ${GLOB_UNITREE_SDK2_THIRDPARTY}
  DESTINATION lib/
)

##################################################
# 项目自定义包含目录
##################################################
include_directories(
  include                     # 项目主头文件目录
  library/matplotlibcpp       # matplotlib C++封装
  library/observation_buffer  # 观测缓冲区
  library/rl_sdk              # 强化学习SDK
  library/loop                # 循环控制相关
)

##################################################
# 构建强化学习SDK库
##################################################
add_library(rl_sdk library/rl_sdk/rl_sdk.cpp)
# 链接Torch和Python库
target_link_libraries(rl_sdk "${TORCH_LIBRARIES}" Python3::Python Python3::Module)
# 设置C++14标准
set_property(TARGET rl_sdk PROPERTY CXX_STANDARD 14)
# 查找Python NumPy扩展
find_package(Python3 COMPONENTS NumPy)

if(Python3_NumPy_FOUND)
# 如果找到NumPy则链接
  target_link_libraries(rl_sdk Python3::NumPy)
else()
# 否则定义WITHOUT_NUMPY宏
  target_compile_definitions(rl_sdk WITHOUT_NUMPY)
endif()

##################################################
# 构建观测缓冲区库
##################################################
add_library(observation_buffer library/observation_buffer/observation_buffer.cpp)
# 链接Torch库
target_link_libraries(observation_buffer "${TORCH_LIBRARIES}")
# 设置C++14标准
set_property(TARGET observation_buffer PROPERTY CXX_STANDARD 14)

##################################################
# 构建仿真器可执行文件
##################################################
add_executable(rl_sim src/rl_sim.cpp )
# 链接必要的库
target_link_libraries(rl_sim
  -pthread            # 线程库
  rl_sdk              # 强化学习SDK
  observation_buffer  # 观测缓冲区
  yaml-cpp            # YAML解析库
)

##################################################
# 构建A1机器人控制可执行文件
##################################################
add_executable(rl_real_a1 src/rl_real_a1.cpp )
target_link_libraries(rl_real_a1
  ${UNITREE_A1_LIBS}
  rl_sdk observation_buffer yaml-cpp
)

##################################################
# 构建Go2机器人控制可执行文件
##################################################
add_executable(rl_real_go2 src/rl_real_go2.cpp )
target_link_libraries(rl_real_go2
  ${UNITREE_GO2_LIBS}
  rl_sdk observation_buffer yaml-cpp
)

##################################################
# 添加ROS2依赖
##################################################
ament_target_dependencies(rl_sim  #添加rl_sim的依赖
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

##################################################
# 安装配置
##################################################
# 安装可执行文件
install(TARGETS
  rl_sim
  rl_sdk
  observation_buffer
  rl_real_a1
  rl_real_go2
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Python脚本
install(PROGRAMS
  scripts/rl_sim.py
  scripts/actuator_net.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件、世界文件和模型文件
install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}/
)
##################################################
# ROS2包配置
##################################################
# 生成ROS2包配置

ament_package()
