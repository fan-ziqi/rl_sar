#ifndef ROBOT_JOINT_CONTROLLER_GROUP_HPP
#define ROBOT_JOINT_CONTROLLER_GROUP_HPP

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <urdf/model.h>
#include "controller_interface/controller_interface.hpp"
#include <std_msgs/msg/float64.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "visibility_control.h"
#if defined(ROS_DISTRO_FOXY)
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#elif defined(ROS_DISTRO_HUMBLE)
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#endif

#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <math.h>
#include <cstdlib>
#include <memory>

#define PosStopF (2.146E+9f)
#define VelStopF (16000.0f)

typedef struct
{
    uint8_t mode;
    double pos;
    double pos_stiffness;
    double vel;
    double vel_stiffness;
    double torque;
} ServoCommand;

namespace robot_joint_controller
{
#if defined(ROS_DISTRO_FOXY)
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
#elif defined(ROS_DISTRO_HUMBLE)
    using CallbackReturn = controller_interface::CallbackReturn;
#endif

class RobotJointControllerGroup : public controller_interface::ControllerInterface
{
public:
    ROBOT_JOINT_CONTROLLER_PUBLIC
    RobotJointControllerGroup();

#if defined(ROS_DISTRO_FOXY)
    ROBOT_JOINT_CONTROLLER_PUBLIC
    controller_interface::return_type update() override;
#elif defined(ROS_DISTRO_HUMBLE)
    ROBOT_JOINT_CONTROLLER_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    ROBOT_JOINT_CONTROLLER_PUBLIC
    CallbackReturn on_init() override;
#endif
    ROBOT_JOINT_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    ROBOT_JOINT_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    ROBOT_JOINT_CONTROLLER_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    ROBOT_JOINT_CONTROLLER_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    ROBOT_JOINT_CONTROLLER_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    void UpdateFunc(const double &period_seconds);
    void SetCommandCallback(const robot_msgs::msg::RobotCommand::SharedPtr msg);
    void PositionLimit(double &position, int &index);
    void VelocityLimit(double &velocity, int &index);
    void EffortLimit(double &effort, int &index);

protected:
    std::string name_space_;
    std::vector<std::string> joint_names_;
    realtime_tools::RealtimeBuffer<robot_msgs::msg::RobotCommand> rt_command_ptr_;
    robot_msgs::msg::RobotCommand last_command_;
    robot_msgs::msg::RobotState last_state_;
    ServoCommand servo_command_;
    std::vector<urdf::JointConstSharedPtr> joints_urdf_;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr robot_description_client_;
    rclcpp::Subscription<robot_msgs::msg::RobotCommand>::SharedPtr joints_command_subscriber_;
    std::shared_ptr<realtime_tools::RealtimePublisher<robot_msgs::msg::RobotState>> controller_state_publisher_;
#if defined(ROS_DISTRO_FOXY)
    rclcpp::Time previous_update_timestamp_{0};
#endif
};

} // namespace robot_joint_controller

#endif // ROBOT_JOINT_CONTROLLER_GROUP_HPP
