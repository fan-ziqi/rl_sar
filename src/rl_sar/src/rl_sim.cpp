/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim.hpp"

// #define PLOT
// #define CSV_LOGGER

RL_Sim::RL_Sim()
#if defined(USE_ROS2)
    : rclcpp::Node("rl_sim_node")
#endif
{
#if defined(USE_ROS1)
    this->ang_vel_type = "ang_vel_world";
    ros::NodeHandle nh;
    nh.param<std::string>("ros_namespace", this->ros_namespace, "");
    nh.param<std::string>("robot_name", this->robot_name, "");
    nh.param<std::string>("config_name", this->default_rl_config, "");
#elif defined(USE_ROS2)
    this->ang_vel_type = "ang_vel_body";
    this->ros_namespace = this->get_namespace();
    // get params from param_node
    param_client = this->create_client<rcl_interfaces::srv::GetParameters>("/param_node/get_parameters");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()) {
            std::cout << LOGGER::ERROR << "Interrupted while waiting for param_node service. Exiting." << std::endl;
            return;
        }
        std::cout << LOGGER::WARNING << "Waiting for param_node service to be available..." << std::endl;
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_name", "gazebo_model_name", "config_name"};
    // Use a timeout for the future
    auto future = param_client->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5));
    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        if (result->values.size() < 3)
        {
            std::cout << LOGGER::ERROR << "Failed to get all parameters from param_node" << std::endl;
        }
        else
        {
            this->robot_name = result->values[0].string_value;
            this->gazebo_model_name = result->values[1].string_value;
            this->default_rl_config = result->values[2].string_value;
            std::cout << LOGGER::INFO << "Get param robot_name: " << this->robot_name << std::endl;
            std::cout << LOGGER::INFO << "Get param gazebo_model_name: " << this->gazebo_model_name << std::endl;
            std::cout << LOGGER::INFO << "Get param config_name: " << this->default_rl_config << std::endl;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "Failed to call param_node service" << std::endl;
    }
#endif

    // read params from yaml
    this->ReadYamlBase(this->robot_name);

    // init torch
    torch::autograd::GradMode::set_enabled(false);
    torch::set_num_threads(4);

    // init robot
#if defined(USE_ROS1)
    this->joint_publishers_commands.resize(this->params.num_of_dofs);
#elif defined(USE_ROS2)
    this->robot_command_publisher_msg.motor_command.resize(this->params.num_of_dofs);
    this->robot_state_subscriber_msg.motor_state.resize(this->params.num_of_dofs);
#endif
    this->InitOutputs();
    this->InitControl();

#if defined(USE_ROS1)
    this->StartJointController(this->ros_namespace, this->params.joint_controller_names);
    // publisher
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const std::string &joint_controller_name = this->params.joint_controller_names[i];
        const std::string topic_name = this->ros_namespace + joint_controller_name + "/command";
        this->joint_publishers[joint_controller_name] =
            nh.advertise<robot_msgs::MotorCommand>(topic_name, 10);
    }

    // subscriber
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Sim::CmdvelCallback, this);
    this->joy_subscriber = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &RL_Sim::JoyCallback, this);
    this->model_state_subscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &RL_Sim::ModelStatesCallback, this);
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const std::string &joint_controller_name = this->params.joint_controller_names[i];
        const std::string topic_name = this->ros_namespace + joint_controller_name + "/state";
        this->joint_subscribers[joint_controller_name] =
            nh.subscribe<robot_msgs::MotorState>(topic_name, 10,
                [this, joint_controller_name](const robot_msgs::MotorState::ConstPtr &msg)
                {
                    this->JointStatesCallback(msg, joint_controller_name);
                }
            );
        this->joint_positions[joint_controller_name] = 0.0;
        this->joint_velocities[joint_controller_name] = 0.0;
        this->joint_efforts[joint_controller_name] = 0.0;
    }

    // service
    nh.param<std::string>("gazebo_model_name", this->gazebo_model_name, "");
    this->gazebo_pause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    this->gazebo_unpause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    this->gazebo_reset_world_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
#elif defined(USE_ROS2)
    this->StartJointController(this->ros_namespace, this->params.joint_names);
    // publisher
    this->robot_command_publisher = this->create_publisher<robot_msgs::msg::RobotCommand>(
        this->ros_namespace + "robot_joint_controller/command", rclcpp::SystemDefaultsQoS());

    // subscriber
    this->cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
    this->joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SystemDefaultsQoS(),
        [this] (const sensor_msgs::msg::Joy::SharedPtr msg) {this->JoyCallback(msg);}
    );
    this->gazebo_imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS(), [this] (const sensor_msgs::msg::Imu::SharedPtr msg) {this->GazeboImuCallback(msg);}
    );
    this->robot_state_subscriber = this->create_subscription<robot_msgs::msg::RobotState>(
        this->ros_namespace + "robot_joint_controller/state", rclcpp::SystemDefaultsQoS(),
        [this] (const robot_msgs::msg::RobotState::SharedPtr msg) {this->RobotStateCallback(msg);}
    );

    // service
    this->gazebo_pause_physics_client = this->create_client<std_srvs::srv::Empty>("/pause_physics");
    this->gazebo_unpause_physics_client = this->create_client<std_srvs::srv::Empty>("/unpause_physics");
    this->gazebo_reset_world_client = this->create_client<std_srvs::srv::Empty>("/reset_world");
#endif

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.num_of_dofs);
    this->plot_target_joint_pos.resize(this->params.num_of_dofs);
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;
}

RL_Sim::~RL_Sim()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::StartJointController(const std::string& ros_namespace, const std::vector<std::string>& names)
{
#if defined(USE_ROS1)
    pid_t pid = fork();
    if (pid == 0)
    {
        std::string cmd = "rosrun controller_manager spawner joint_state_controller ";
        for (const auto& name : names)
        {
            cmd += name + " ";
        }
        cmd += "__ns:=" + ros_namespace;
        // cmd += " > /dev/null 2>&1";  // Comment this line to see the output
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
#elif defined(USE_ROS2)
    const char* ros_distro = std::getenv("ROS_DISTRO");
    std::string spawner = (ros_distro && std::string(ros_distro) == "foxy") ? "spawner.py" : "spawner";

    std::filesystem::path tmp_path = std::filesystem::temp_directory_path() / "robot_joint_controller_params.yaml";
    {
        std::ofstream tmp_file(tmp_path);
        if (!tmp_file)
        {
            throw std::runtime_error("Failed to create temporary parameter file");
        }

        tmp_file << "/controller_manager:\n";
        tmp_file << "    ros__parameters:\n";
        tmp_file << "        update_rate: 1000  # Hz\n";
        tmp_file << "        use_sim_time: true  # If running in simulation\n\n";
        tmp_file << "        robot_joint_controller:\n";
        tmp_file << "            type: robot_joint_controller/RobotJointControllerGroup\n\n";

        tmp_file << "/robot_joint_controller:\n";
        tmp_file << "    ros__parameters:\n";
        tmp_file << "        joints:\n";

        for (const auto& name : names)
        {
            tmp_file << "            - " << name << "\n";
        }
    }

    pid_t pid = fork();
    if (pid == 0)
    {
        std::string cmd = "ros2 run controller_manager " + spawner + " robot_joint_controller ";
        cmd += "-p " + tmp_path.string() + " ";
        // cmd += " > /dev/null 2>&1";  // Comment this line to see the output
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
    else if (pid > 0)
    {
        int status;
        waitpid(pid, &status, 0);
        std::filesystem::remove(tmp_path);

        if (WIFEXITED(status) && WEXITSTATUS(status) != 0)
        {
            throw std::runtime_error("Failed to start joint controller");
        }
    }
    else
    {
        throw std::runtime_error("fork() failed");
    }
#endif
}

void RL_Sim::GetState(RobotState<double> *state)
{
#if defined(USE_ROS1)
    const auto &orientation = this->pose.orientation;
    const auto &angular_velocity = this->vel.angular;
#elif defined(USE_ROS2)
    const auto &orientation = this->gazebo_imu.orientation;
    const auto &angular_velocity = this->gazebo_imu.angular_velocity;
#endif

    if (this->params.framework == "isaacgym")
    {
        state->imu.quaternion[3] = orientation.w;
        state->imu.quaternion[0] = orientation.x;
        state->imu.quaternion[1] = orientation.y;
        state->imu.quaternion[2] = orientation.z;
    }
    else if (this->params.framework == "isaacsim")
    {
        state->imu.quaternion[0] = orientation.w;
        state->imu.quaternion[1] = orientation.x;
        state->imu.quaternion[2] = orientation.y;
        state->imu.quaternion[3] = orientation.z;
    }

    state->imu.gyroscope[0] = angular_velocity.x;
    state->imu.gyroscope[1] = angular_velocity.y;
    state->imu.gyroscope[2] = angular_velocity.z;

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
#if defined(USE_ROS1)
        state->motor_state.q[i] = this->joint_positions[this->params.joint_controller_names[i]];
        state->motor_state.dq[i] = this->joint_velocities[this->params.joint_controller_names[i]];
        state->motor_state.tau_est[i] = this->joint_efforts[this->params.joint_controller_names[i]];
#elif defined(USE_ROS2)
        state->motor_state.q[i] = this->robot_state_subscriber_msg.motor_state[this->params.state_mapping[i]].q;
        state->motor_state.dq[i] = this->robot_state_subscriber_msg.motor_state[this->params.state_mapping[i]].dq;
        state->motor_state.tau_est[i] = this->robot_state_subscriber_msg.motor_state[this->params.state_mapping[i]].tau_est;
#endif
    }
}

void RL_Sim::SetCommand(const RobotCommand<double> *command)
{
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
#if defined(USE_ROS1)
        this->joint_publishers_commands[i].q = command->motor_command.q[i];
        this->joint_publishers_commands[i].dq = command->motor_command.dq[i];
        this->joint_publishers_commands[i].kp = command->motor_command.kp[i];
        this->joint_publishers_commands[i].kd = command->motor_command.kd[i];
        this->joint_publishers_commands[i].tau = command->motor_command.tau[i];
#elif defined(USE_ROS2)
        this->robot_command_publisher_msg.motor_command[i].q = command->motor_command.q[this->params.command_mapping[i]];
        this->robot_command_publisher_msg.motor_command[i].dq = command->motor_command.dq[this->params.command_mapping[i]];
        this->robot_command_publisher_msg.motor_command[i].kp = command->motor_command.kp[this->params.command_mapping[i]];
        this->robot_command_publisher_msg.motor_command[i].kd = command->motor_command.kd[this->params.command_mapping[i]];
        this->robot_command_publisher_msg.motor_command[i].tau = command->motor_command.tau[this->params.command_mapping[i]];
#endif
    }

#if defined(USE_ROS1)
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->joint_publishers[this->params.joint_controller_names[i]].publish(this->joint_publishers_commands[i]);
    }
#elif defined(USE_ROS2)
    this->robot_command_publisher->publish(this->robot_command_publisher_msg);
#endif
}

void RL_Sim::RobotControl()
{

    if (this->control.control_state == STATE_RESET_SIMULATION)
    {
#if defined(USE_ROS1)
        std_srvs::Empty empty;
        this->gazebo_reset_world_client.call(empty);
#elif defined(USE_ROS2)
        auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
#endif
        this->control.control_state = this->control.last_control_state;
    }
    if (this->control.control_state == STATE_TOGGLE_SIMULATION)
    {
        if (simulation_running)
        {
#if defined(USE_ROS1)
            std_srvs::Empty empty;
            this->gazebo_pause_physics_client.call(empty);
#elif defined(USE_ROS2)
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_pause_physics_client->async_send_request(empty_request);
#endif
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
#if defined(USE_ROS1)
            std_srvs::Empty empty;
            this->gazebo_unpause_physics_client.call(empty);
#elif defined(USE_ROS2)
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_unpause_physics_client->async_send_request(empty_request);
#endif
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
        this->control.control_state = this->control.last_control_state;
    }

    if (simulation_running)
    {
        this->motiontime++;
        this->GetState(&this->robot_state);
        this->StateController(&this->robot_state, &this->robot_command);
        this->SetCommand(&this->robot_command);
    }
}

#if defined(USE_ROS1)
void RL_Sim::ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    this->vel = msg->twist[2];
    this->pose = msg->pose[2];
}
#elif defined(USE_ROS2)
void RL_Sim::GazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    this->gazebo_imu = *msg;
}
#endif

void RL_Sim::CmdvelCallback(
#if defined(USE_ROS1)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}

void RL_Sim::JoyCallback(
#if defined(USE_ROS1)
    const sensor_msgs::Joy::ConstPtr &msg
#elif defined(USE_ROS2)
    const sensor_msgs::msg::Joy::SharedPtr msg
#endif
)
{
    this->joy_msg = *msg;

    // joystick control
    // Description of buttons and axes(F710):
    // |__ buttons[]: A=0, B=1, X=2, Y=3, LB=4, RB=5, back=6, start=7, power=8, stickL=9, stickR=10
    // |__ axes[]: Lx=0, Ly=1, Rx=3, Ry=4, LT=2, RT=5
    if (this->joy_msg.buttons[5])
    {
        if (this->joy_msg.buttons[3]) // RB+Y
        {
            this->control.SetControlState(STATE_POS_GETUP);
        }
        else if (this->joy_msg.buttons[0]) // RB+A
        {
            this->control.SetControlState(STATE_POS_GETDOWN);
        }
        else if (this->joy_msg.buttons[1]) // RB+B
        {
            this->control.SetControlState(STATE_RL_LOCOMOTION);
        }
        else if (this->joy_msg.buttons[2]) // RB+X
        {
            this->control.SetControlState(STATE_RESET_SIMULATION);
        }
        else if (this->joy_msg.axes[7] < 0) // DOWN
        {
            this->control.SetControlState(STATE_RL_NAVIGATION);
        }
    }
    if (this->joy_msg.buttons[4]) // LB
    {
        this->control.SetControlState(STATE_TOGGLE_SIMULATION);
    }

    this->control.x = this->joy_msg.axes[1] * 1.5; // Ly
    this->control.y = this->joy_msg.axes[0] * 1.5; // Lx
    this->control.yaw = this->joy_msg.axes[3] * 1.5; // Rx
}

#if defined(USE_ROS1)
void RL_Sim::JointStatesCallback(const robot_msgs::MotorState::ConstPtr &msg, const std::string &joint_controller_name)
{
    this->joint_positions[joint_controller_name] = msg->q;
    this->joint_velocities[joint_controller_name] = msg->dq;
    this->joint_efforts[joint_controller_name] = msg->tau_est;
}
#elif defined(USE_ROS2)
void RL_Sim::RobotStateCallback(const robot_msgs::msg::RobotState::SharedPtr msg)
{
    this->robot_state_subscriber_msg = *msg;
}
#endif

void RL_Sim::RunModel()
{
    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;
        // this->obs.lin_vel = torch::tensor({{this->vel.linear.x, this->vel.linear.y, this->vel.linear.z}});
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        if (this->fsm._currentState->getStateName() == "RLFSMStateRL_Navigation")
        {
            this->obs.commands = torch::tensor({{this->cmd_vel.linear.x, this->cmd_vel.linear.y, this->cmd_vel.angular.z}});
        }
        else
        {
            this->obs.commands = torch::tensor({{this->control.x, this->control.y, this->control.yaw}});
        }
        this->obs.base_quat = torch::tensor(this->robot_state.imu.quaternion).unsqueeze(0);
        this->obs.dof_pos = torch::tensor(this->robot_state.motor_state.q).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);
        this->obs.dof_vel = torch::tensor(this->robot_state.motor_state.dq).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (this->output_dof_pos.defined() && this->output_dof_pos.numel() > 0)
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (this->output_dof_vel.defined() && this->output_dof_vel.numel() > 0)
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (this->output_dof_tau.defined() && this->output_dof_tau.numel() > 0)
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);

#ifdef CSV_LOGGER
        torch::Tensor tau_est = torch::zeros({1, this->params.num_of_dofs});
        for (int i = 0; i < this->params.num_of_dofs; ++i)
        {
            tau_est[0][i] = this->joint_efforts[this->params.joint_controller_names[i]];
        }
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

torch::Tensor RL_Sim::Forward()
{
    torch::autograd::GradMode::set_enabled(false);

    torch::Tensor clamped_obs = this->ComputeObservation();

    torch::Tensor actions;
    if (this->params.observations_history.size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.observations_history);
        actions = this->model.forward({this->history_obs}).toTensor();
    }
    else
    {
        actions = this->model.forward({clamped_obs}).toTensor();
    }

    if (this->params.clip_actions_upper.numel() != 0 && this->params.clip_actions_lower.numel() != 0)
    {
        return torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);
    }
    else
    {
        return actions;
    }
}

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
#if defined(USE_ROS1)
        this->plot_real_joint_pos[i].push_back(this->joint_positions[this->params.joint_controller_names[i]]);
        this->plot_target_joint_pos[i].push_back(this->joint_publishers_commands[i].q);
#elif defined(USE_ROS2)
        this->plot_real_joint_pos[i].push_back(this->robot_state_subscriber_msg.motor_state[i].q);
        this->plot_target_joint_pos[i].push_back(this->robot_command_publisher_msg.motor_command[i].q);
#endif
        plt::subplot(4, 3, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}

#if defined(USE_ROS1)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS1)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Sim rl_sar;
    ros::spin();
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RL_Sim>());
    rclcpp::shutdown();
#endif
    return 0;
}
