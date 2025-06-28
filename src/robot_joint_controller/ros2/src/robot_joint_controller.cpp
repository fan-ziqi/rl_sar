#include "robot_joint_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace robot_joint_controller
{
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;

RobotJointController::RobotJointController()
    : controller_interface::ControllerInterface(),
        joints_command_subscriber_(nullptr),
        controller_state_publisher_(nullptr)
{
    memset(&last_command_, 0, sizeof(robot_msgs::msg::MotorCommand));
    memset(&last_state_, 0, sizeof(robot_msgs::msg::MotorState));
    memset(&servo_command_, 0, sizeof(ServoCommand));
}

#if defined(ROS_DISTRO_HUMBLE)
CallbackReturn RobotJointController::on_init()
{
    return CallbackReturn::SUCCESS;
}
#endif

CallbackReturn RobotJointController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    name_space_ = get_node()->get_namespace();

    if (!get_node()->get_parameter("joint", joint_name_))
    {
        RCLCPP_ERROR(get_node()->get_logger(), "No joint given in namespace: '%s'", name_space_.c_str());
        return CallbackReturn::ERROR;
    }
    else
    {
        RCLCPP_WARN(get_node()->get_logger(), "joint_name_: %s", joint_name_.c_str());
    }

    robot_description_client_ = get_node()->create_client<rcl_interfaces::srv::GetParameters>("/robot_state_publisher/get_parameters");

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("robot_description_");

    while (!robot_description_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_WARN(get_node()->get_logger(), "Service not available, waiting again...");
    }

	auto response_received_callback = [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future)
    {
        std::string robot_description = future.get()->values[0].string_value;
        urdf::Model urdf;
        if (!urdf.initString(robot_description))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
        }

        joints_urdf_ = urdf.getJoint(joint_name_);
        if (!joints_urdf_)
        {
            RCLCPP_ERROR(get_node()->get_logger(),"Could not find joint '%s' in urdf", joint_name_.c_str());
        }
	};
	auto future_result = robot_description_client_->async_send_request(request, response_received_callback);

    // Start command subscriber
    joints_command_subscriber_ = get_node()->create_subscription<robot_msgs::msg::MotorCommand>(
        "~/command", rclcpp::SystemDefaultsQoS(), std::bind(&RobotJointController::SetCommandCallback, this, std::placeholders::_1));

    // Start realtime state publisher
    controller_state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<robot_msgs::msg::MotorState>>(
        get_node()->create_publisher<robot_msgs::msg::MotorState>(/*name_space_ + */"~/state", rclcpp::SystemDefaultsQoS()));

#if defined(ROS_DISTRO_FOXY)
    previous_update_timestamp_ = get_node()->get_clock()->now();
#endif

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotJointController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.push_back(joint_name_ + "/" + HW_IF_EFFORT);

    return command_interfaces_config;

}

controller_interface::InterfaceConfiguration RobotJointController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.push_back(joint_name_ + "/" + HW_IF_POSITION);
    state_interfaces_config.names.push_back(joint_name_ + "/" + HW_IF_EFFORT);

    return state_interfaces_config;
}

CallbackReturn RobotJointController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    double init_pos = state_interfaces_[0].get_value();
    last_command_.q = init_pos;
    last_state_.q = init_pos;
    last_command_.dq = 0;
    last_state_.dq = 0;
    last_command_.tau = 0;
    last_state_.tau_est = 0;

    // reset command buffer if a command came through callback when controller was inactive
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<robot_msgs::msg::MotorCommand>();

    return CallbackReturn::SUCCESS;
}

CallbackReturn RobotJointController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    // reset command buffer
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<robot_msgs::msg::MotorCommand>();
    return CallbackReturn::SUCCESS;
}

#if defined(ROS_DISTRO_FOXY)
controller_interface::return_type RobotJointController::update()
{
    const auto current_time = get_node()->get_clock()->now();
    const auto period_seconds = (current_time - previous_update_timestamp_).seconds();
    previous_update_timestamp_ = current_time;
    auto joint_command = rt_command_ptr_.readFromRT();
    // no command received yet
    if (!joint_command)
    {
        return controller_interface::return_type::OK;
    }
    last_command_ = *(joint_command);
    UpdateFunc(period_seconds);
    return controller_interface::return_type::OK;
}
#elif defined(ROS_DISTRO_HUMBLE)
controller_interface::return_type RobotJointController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    const auto period_seconds = period.seconds();
    auto joint_command = rt_command_ptr_.readFromRT();
    // no command received yet
    if (!joint_command)
    {
        return controller_interface::return_type::OK;
    }
    last_command_ = *(joint_command);
    UpdateFunc(period_seconds);
    return controller_interface::return_type::OK;
}
#endif

void RobotJointController::UpdateFunc(const double &period_seconds)
{
    double currentPos = 0.0;
    double currentVel = 0.0;
    double calcTorque = 0.0;

    // set command data
    servo_command_.pos = last_command_.q;
    PositionLimit(servo_command_.pos);
    servo_command_.pos_stiffness = last_command_.kp;
    if (fabs(last_command_.q - PosStopF) < 0.00001)
    {
        servo_command_.pos_stiffness = 0;
    }
    servo_command_.vel = last_command_.dq;
    VelocityLimit(servo_command_.vel);
    servo_command_.vel_stiffness = last_command_.kd;
    if (fabs(last_command_.dq - VelStopF) < 0.00001)
    {
        servo_command_.vel_stiffness = 0;
    }
    servo_command_.torque = last_command_.tau;
    EffortLimit(servo_command_.torque);

    currentPos = state_interfaces_[0].get_value();
    currentVel = (currentPos - (double)(last_state_.q)) / period_seconds;
    calcTorque = servo_command_.pos_stiffness * (servo_command_.pos - currentPos) + servo_command_.vel_stiffness * (servo_command_.vel - currentVel) + servo_command_.torque;
    EffortLimit(calcTorque);

    command_interfaces_[0].set_value(calcTorque);

    last_state_.q = currentPos;
    last_state_.dq = currentVel;
    last_state_.tau_est = state_interfaces_[1].get_value();

    // publish state
    if (controller_state_publisher_ && controller_state_publisher_->trylock())
    {
        controller_state_publisher_->msg_ = last_state_;
        controller_state_publisher_->unlockAndPublish();
    }
}

void RobotJointController::SetCommandCallback(const robot_msgs::msg::MotorCommand::SharedPtr msg)
{
    last_command_ = *msg;
    rt_command_ptr_.writeFromNonRT(last_command_);
}

void RobotJointController::PositionLimit(double &position)
{
    std::clamp(position, joints_urdf_->limits->lower, joints_urdf_->limits->upper);
}

void RobotJointController::VelocityLimit(double &velocity)
{
    std::clamp(velocity, -joints_urdf_->limits->velocity, joints_urdf_->limits->velocity);
}

void RobotJointController::EffortLimit(double &effort)
{
    std::clamp(effort, -joints_urdf_->limits->effort, joints_urdf_->limits->effort);
}

} // namespace robot_joint_controller

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(robot_joint_controller::RobotJointController, controller_interface::ControllerInterface)
