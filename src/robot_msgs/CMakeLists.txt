cmake_minimum_required(VERSION 3.5)
project(robot_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/RobotCommand.msg"
  "msg/RobotState.msg"
  "msg/IMU.msg"
)

ament_package()
