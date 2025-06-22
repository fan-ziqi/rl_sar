#include "robot_joint_controller_group.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace robot_joint_controller
{
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;

RobotJointControllerGroup::RobotJointControllerGroup()
    : controller_interface::ControllerInterface(),
        joints_command_subscriber_(nullptr),
        controller_state_publisher_(nullptr)
{
    memset(&servo_command_, 0, sizeof(ServoCommand));
}

#if defined(ROS_DISTRO_HUMBLE)
CallbackReturn RobotJointControllerGroup::on_init()
{
    return CallbackReturn::SUCCESS;
}
#endif

CallbackReturn RobotJointControllerGroup::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    name_space_ = get_node()->get_namespace();

    // joint_names_ = get_node()->get_parameter("joints").as_string_array();

    // if (joint_names_.empty())
    // {
    //     RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    //     return CallbackReturn::ERROR;
    // }
    // else
    // {
    //     for (const auto& joint_name : joint_names_)
    //     {
    //         RCLCPP_WARN(get_node()->get_logger(), "joint_name: %s", joint_name.c_str());
    //     }
    // }

    while (rclcpp::ok())
    {
        if (!get_node()->has_parameter("joints"))
        {
            RCLCPP_WARN(get_node()->get_logger(), "Waiting for 'joints' parameter to be set...");
            std::this_thread::sleep_for(std::chrono::seconds(1));  // 每秒检查一次
            continue;
        }

        joint_names_ = get_node()->get_parameter("joints").as_string_array();

        if (joint_names_.empty())
        {
            RCLCPP_WARN(get_node()->get_logger(), "'joints' parameter is empty, waiting...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        else
        {
            for (const auto& joint_name : joint_names_)
            {
                RCLCPP_INFO(get_node()->get_logger(), "Configured joint: %s", joint_name.c_str());
            }
            break;
        }
    }

    robot_description_client_ = get_node()->create_client<rcl_interfaces::srv::GetParameters>("/robot_state_publisher/get_parameters");

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("robot_description");

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

        for (const auto& joint_name : joint_names_)
        {
            auto joint_urdf = urdf.getJoint(joint_name);
            if (!joint_urdf)
            {
                RCLCPP_ERROR(get_node()->get_logger(),"Could not find joint '%s' in urdf", joint_name.c_str());
            }
            if (joint_urdf)
            {
                joints_urdf_.push_back(joint_urdf);
            }
        }
	};
	auto future_result = robot_description_client_->async_send_request(request, response_received_callback);

    // Start command subscriber
    joints_command_subscriber_ = get_node()->create_subscription<robot_msgs::msg::RobotCommand>(
        "~/command", rclcpp::SystemDefaultsQoS(), std::bind(&RobotJointControllerGroup::SetCommandCallback, this, std::placeholders::_1));

    // Start realtime state publisher
    controller_state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<robot_msgs::msg::RobotState>>(
        get_node()->create_publisher<robot_msgs::msg::RobotState>(/*name_space_ + */"~/state", rclcpp::SystemDefaultsQoS()));

#if defined(ROS_DISTRO_FOXY)
    previous_update_timestamp_ = get_node()->get_clock()->now();
#endif

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotJointControllerGroup::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & joint_name : joint_names_)
    {
        command_interfaces_config.names.push_back(joint_name + "/" + HW_IF_EFFORT);
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration RobotJointControllerGroup::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & joint_name : joint_names_)
    {
        state_interfaces_config.names.push_back(joint_name + "/" + HW_IF_POSITION);
        state_interfaces_config.names.push_back(joint_name + "/" + HW_IF_EFFORT);
    }

    return state_interfaces_config;
}

CallbackReturn RobotJointControllerGroup::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    last_command_ = robot_msgs::msg::RobotCommand();
    last_command_.motor_command.resize(joint_names_.size());
    last_state_ = robot_msgs::msg::RobotState();
    last_state_.motor_state.resize(joint_names_.size());

    for (int index = 0; index < joint_names_.size(); ++index)
    {
        double init_pos = state_interfaces_[index * 2].get_value();
        last_command_.motor_command[index].q = init_pos;
        last_state_.motor_state[index].q = init_pos;
        last_command_.motor_command[index].dq = 0;
        last_state_.motor_state[index].dq = 0;
        last_command_.motor_command[index].tau = 0;
        last_state_.motor_state[index].tau_est = 0;
    }

    // reset command buffer if a command came through callback when controller was inactive
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<robot_msgs::msg::RobotCommand>();
    rt_command_ptr_.writeFromNonRT(last_command_);

    return CallbackReturn::SUCCESS;
}

CallbackReturn RobotJointControllerGroup::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    last_command_ = robot_msgs::msg::RobotCommand();
    last_command_.motor_command.resize(joint_names_.size());
    last_state_ = robot_msgs::msg::RobotState();
    last_state_.motor_state.resize(joint_names_.size());

    // reset command buffer
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<robot_msgs::msg::RobotCommand>();
    rt_command_ptr_.writeFromNonRT(last_command_);
    return CallbackReturn::SUCCESS;
}

#if defined(ROS_DISTRO_FOXY)
controller_interface::return_type RobotJointControllerGroup::update()
{
    const auto current_time = get_node()->get_clock()->now();
    const auto period_seconds = (current_time - previous_update_timestamp_).seconds();
    previous_update_timestamp_ = current_time;
    auto joint_commands = rt_command_ptr_.readFromRT();
    // no command received yet
    if (!joint_commands)
    {
        return controller_interface::return_type::OK;
    }
    if (joint_commands->motor_command.size() != joint_names_.size())
    {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "command size (%zu) does not match number of interfaces (%zu)",
            joint_commands->motor_command.size(), joint_names_.size());
        return controller_interface::return_type::ERROR;
    }
    last_command_ = *(joint_commands);
    UpdateFunc(period_seconds);
    return controller_interface::return_type::OK;
}
#elif defined(ROS_DISTRO_HUMBLE)
controller_interface::return_type RobotJointControllerGroup::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    const auto period_seconds = period.seconds();
    auto joint_commands = rt_command_ptr_.readFromRT();
    // no command received yet
    if (!joint_commands)
    {
        return controller_interface::return_type::OK;
    }
    if (joint_commands->motor_command.size() != joint_names_.size())
    {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "command size (%zu) does not match number of interfaces (%zu)",
            joint_commands->motor_command.size(), joint_names_.size());
        return controller_interface::return_type::ERROR;
    }
    last_command_ = *(joint_commands);
    UpdateFunc(period_seconds);
    return controller_interface::return_type::OK;
}
#endif

void RobotJointControllerGroup::UpdateFunc(const double &period_seconds)
{
    std::vector<double> currentPos(joint_names_.size(), 0.0);
    std::vector<double> currentVel(joint_names_.size(), 0.0);
    std::vector<double> calcTorque(joint_names_.size(), 0.0);

    for (int index = 0; index < joint_names_.size(); ++index)
    {
        // set command data
        servo_command_.pos = last_command_.motor_command[index].q;
        PositionLimit(servo_command_.pos, index);
        servo_command_.pos_stiffness = last_command_.motor_command[index].kp;
        if (fabs(last_command_.motor_command[index].q - PosStopF) < 0.00001)
        {
            servo_command_.pos_stiffness = 0;
        }
        servo_command_.vel = last_command_.motor_command[index].dq;
        VelocityLimit(servo_command_.vel, index);
        servo_command_.vel_stiffness = last_command_.motor_command[index].kd;
        if (fabs(last_command_.motor_command[index].dq - VelStopF) < 0.00001)
        {
            servo_command_.vel_stiffness = 0;
        }
        servo_command_.torque = last_command_.motor_command[index].tau;
        EffortLimit(servo_command_.torque, index);

        currentPos[index] = state_interfaces_[index * 2].get_value();
        currentVel[index] = (currentPos[index] - (double)(last_state_.motor_state[index].q)) / period_seconds;
        calcTorque[index] = servo_command_.pos_stiffness * (servo_command_.pos - currentPos[index]) + servo_command_.vel_stiffness * (servo_command_.vel - currentVel[index]) + servo_command_.torque;
        EffortLimit(calcTorque[index], index);

        command_interfaces_[index].set_value(calcTorque[index]);

        last_state_.motor_state[index].q = currentPos[index];
        last_state_.motor_state[index].dq = currentVel[index];
        last_state_.motor_state[index].tau_est = state_interfaces_[index + 1].get_value();
    }

    // publish state
    if (controller_state_publisher_ && controller_state_publisher_->trylock())
    {
        controller_state_publisher_->msg_.motor_state = last_state_.motor_state;
        controller_state_publisher_->unlockAndPublish();
    }
}

void RobotJointControllerGroup::SetCommandCallback(const robot_msgs::msg::RobotCommand::SharedPtr msg)
{
    last_command_ = *msg;
    rt_command_ptr_.writeFromNonRT(last_command_);
}

void RobotJointControllerGroup::PositionLimit(double &position, int &index)
{
    std::clamp(position, joints_urdf_[index]->limits->lower, joints_urdf_[index]->limits->upper);
}

void RobotJointControllerGroup::VelocityLimit(double &velocity, int &index)
{
    std::clamp(velocity, -joints_urdf_[index]->limits->velocity, joints_urdf_[index]->limits->velocity);
}

void RobotJointControllerGroup::EffortLimit(double &effort, int &index)
{
    std::clamp(effort, -joints_urdf_[index]->limits->effort, joints_urdf_[index]->limits->effort);
}

} // namespace robot_joint_controller

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(robot_joint_controller::RobotJointControllerGroup, controller_interface::ControllerInterface)
