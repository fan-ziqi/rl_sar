/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_lite3.hpp"

RL_Real::RL_Real()
#if defined(USE_ROS2) && defined(USE_ROS)
    : rclcpp::Node("rl_real_node")
#endif
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    this->cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
#endif

    // read params from yaml
    this->ang_vel_type = "ang_vel_world";
    this->robot_name = "lite3";
    this->ReadYamlBase(this->robot_name);

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init torch
    torch::autograd::GradMode::set_enabled(false);
    torch::set_num_threads(4);


    // Network init
    int local_port = 43987;
    int robot_port = 43893;
    std::string robot_ip = "192.168.2.1";
    // init robot
    this->receiver_ = new Receiver();
    this->sender_ = new Sender(robot_ip, robot_port);
    this->sender_->RobotStateInit();
    this->InitOutputs();
    this->InitControl();
    this->receiver_->StartWork();
    this->robot_data_ = &(receiver_->GetState());

    // init gamepad
    this->gamepad_ptr_ = std::make_shared<RetroidGamepad>(12121);
    this->first_flag_ = true;
    this->gamepad_ptr_->StartDataThread();

    // loop
    this->loop_udpRecv = std::make_shared<LoopFunc>("loop_udpRecv", 0.002, std::bind(&RL_Real::UDPRecv, this), 3);
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Real::RunModel, this));
    this->loop_udpRecv->start();
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.num_of_dofs);
    this->plot_target_joint_pos.resize(this->params.num_of_dofs);
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real::~RL_Real()
{
    this->loop_udpRecv->shutdown();
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    this->gamepad_ptr_->StopDataThread();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    this->rt_keys_ = this->gamepad_ptr_->GetKeys();
    if(this->first_flag_){
        this->rt_keys_record_ = this->rt_keys_;
        this->first_flag_ = false;
    }
    if (this->rt_keys_.A != this->rt_keys_record_.A) this->control.SetGamepad(Input::Gamepad::A);
    if (this->rt_keys_.B != this->rt_keys_record_.B) this->control.SetGamepad(Input::Gamepad::B);
    if (this->rt_keys_.X != this->rt_keys_record_.X) this->control.SetGamepad(Input::Gamepad::X);
    if (this->rt_keys_.Y != this->rt_keys_record_.Y) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->rt_keys_.L1 != this->rt_keys_record_.L1) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->rt_keys_.R1 != this->rt_keys_record_.R1) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->rt_keys_.left_axis_button != this->rt_keys_record_.left_axis_button) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->rt_keys_.right_axis_button != this->rt_keys_record_.right_axis_button) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->rt_keys_.up != this->rt_keys_record_.up) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->rt_keys_.down != this->rt_keys_record_.down) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->rt_keys_.left != this->rt_keys_record_.left) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->rt_keys_.right != this->rt_keys_record_.right) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if ((this->rt_keys_.L1 != this->rt_keys_record_.L1)&&(this->rt_keys_.A != this->rt_keys_record_.A)) this->control.SetGamepad(Input::Gamepad::LB_A);
    if ((this->rt_keys_.L1 != this->rt_keys_record_.L1)&&(this->rt_keys_.B != this->rt_keys_record_.B)) this->control.SetGamepad(Input::Gamepad::LB_B);
    if ((this->rt_keys_.L1 != this->rt_keys_record_.L1)&&(this->rt_keys_.X != this->rt_keys_record_.X)) this->control.SetGamepad(Input::Gamepad::LB_X);
    if ((this->rt_keys_.L1 != this->rt_keys_record_.L1)&&(this->rt_keys_.Y != this->rt_keys_record_.Y)) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if ((this->rt_keys_.L1 != this->rt_keys_record_.L1)&&(this->rt_keys_.left_axis_button != this->rt_keys_record_.left_axis_button)) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if ((this->rt_keys_.L1 != this->rt_keys_record_.L1)&&(this->rt_keys_.right_axis_button != this->rt_keys_record_.right_axis_button)) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if ((this->rt_keys_.L1 != this->rt_keys_record_.L1)&&(this->rt_keys_.up != this->rt_keys_record_.up)) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if ((this->rt_keys_.L1 != this->rt_keys_record_.L1)&&(this->rt_keys_.down != this->rt_keys_record_.down)) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if ((this->rt_keys_.L1 != this->rt_keys_record_.L1)&&(this->rt_keys_.left != this->rt_keys_record_.left)) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if ((this->rt_keys_.L1 != this->rt_keys_record_.L1)&&(this->rt_keys_.right != this->rt_keys_record_.right)) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if ((this->rt_keys_.R1 != this->rt_keys_record_.R1)&&(this->rt_keys_.A != this->rt_keys_record_.A)) this->control.SetGamepad(Input::Gamepad::RB_A);
    if ((this->rt_keys_.R1 != this->rt_keys_record_.R1)&&(this->rt_keys_.B != this->rt_keys_record_.B)) this->control.SetGamepad(Input::Gamepad::RB_B);
    if ((this->rt_keys_.R1 != this->rt_keys_record_.R1)&&(this->rt_keys_.X != this->rt_keys_record_.X)) this->control.SetGamepad(Input::Gamepad::RB_X);
    if ((this->rt_keys_.R1 != this->rt_keys_record_.R1)&&(this->rt_keys_.Y != this->rt_keys_record_.Y)) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if ((this->rt_keys_.R1 != this->rt_keys_record_.R1)&&(this->rt_keys_.left_axis_button != this->rt_keys_record_.left_axis_button)) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if ((this->rt_keys_.R1 != this->rt_keys_record_.R1)&&(this->rt_keys_.right_axis_button != this->rt_keys_record_.right_axis_button)) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if ((this->rt_keys_.R1 != this->rt_keys_record_.R1)&&(this->rt_keys_.up != this->rt_keys_record_.up)) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if ((this->rt_keys_.R1 != this->rt_keys_record_.R1)&&(this->rt_keys_.down != this->rt_keys_record_.down)) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if ((this->rt_keys_.R1 != this->rt_keys_record_.R1)&&(this->rt_keys_.left != this->rt_keys_record_.left)) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if ((this->rt_keys_.R1 != this->rt_keys_record_.R1)&&(this->rt_keys_.right != this->rt_keys_record_.right)) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if ((this->rt_keys_.R1 != this->rt_keys_record_.R1)&&(this->rt_keys_.R1 != this->rt_keys_record_.R1)) this->control.SetGamepad(Input::Gamepad::LB_RB);

    this->control.x = this->rt_keys_.left_axis_y;
    this->control.y = -this->rt_keys_.left_axis_x;
    this->control.yaw = -this->rt_keys_.right_axis_x;

    float q[4];
    EulerToQuaternion(this->robot_data_->imu.angle_roll, this->robot_data_->imu.angle_pitch, this->robot_data_->imu.angle_yaw, q);

    state->imu.quaternion[0] = q[0]; // w
    state->imu.quaternion[1] = q[1]; // x
    state->imu.quaternion[2] = q[2]; // y
    state->imu.quaternion[3] = q[3]; // z

    state->imu.gyroscope[0] = this->robot_data_->imu.angular_velocity_roll;
    state->imu.gyroscope[1] = this->robot_data_->imu.angular_velocity_pitch;
    state->imu.gyroscope[2] = this->robot_data_->imu.angular_velocity_yaw;

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        state->motor_state.q[i] = this->robot_data_->joint_data.joint_data[this->params.joint_mapping[i]].position;
        state->motor_state.dq[i] = this->robot_data_->joint_data.joint_data[this->params.joint_mapping[i]].velocity;
        state->motor_state.tau_est[i] = this->robot_data_->joint_data.joint_data[this->params.joint_mapping[i]].torque;
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->robot_joint_cmd_.joint_cmd[this->params.joint_mapping[i]].position = command->motor_command.q[i];
        this->robot_joint_cmd_.joint_cmd[this->params.joint_mapping[i]].velocity = command->motor_command.dq[i];
        this->robot_joint_cmd_.joint_cmd[this->params.joint_mapping[i]].kp = command->motor_command.kp[i];
        this->robot_joint_cmd_.joint_cmd[this->params.joint_mapping[i]].kd = command->motor_command.kd[i];
        this->robot_joint_cmd_.joint_cmd[this->params.joint_mapping[i]].torque = command->motor_command.tau[i];
    }

    this->sender_->SendCmd(robot_joint_cmd_);
}

void RL_Real::RobotControl()
{
    this->motiontime++;

    if (this->control.current_keyboard == Input::Keyboard::W)
    {
        this->control.x += 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::S)
    {
        this->control.x -= 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::A)
    {
        this->control.y += 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::D)
    {
        this->control.y -= 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Q)
    {
        this->control.yaw += 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::E)
    {
        this->control.yaw -= 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0;
        this->control.y = 0;
        this->control.yaw = 0;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
        this->control.current_keyboard = this->control.last_keyboard;
        this->control.current_gamepad = this->control.last_gamepad;
    }

    this->GetState(&this->robot_state);
    this->StateController(&this->robot_state, &this->robot_command);
    this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        if (this->control.navigation_mode)
        {
#if !defined(USE_CMAKE) && defined(USE_ROS)
            this->obs.commands = torch::tensor({{this->cmd_vel.linear.x, this->cmd_vel.linear.y, this->cmd_vel.angular.z}});
#endif
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
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        torch::Tensor tau_est = torch::tensor(this->robot_state.motor_state.tau_est).unsqueeze(0);
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

torch::Tensor RL_Real::Forward()
{
    torch::autograd::GradMode::set_enabled(false);

    torch::Tensor clamped_obs = this->ComputeObservation();

    torch::Tensor actions;
    if (!this->params.observations_history.empty())
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

void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->robot_data_->joint_data.joint_data[this->params.joint_mapping[i]].position);
        this->plot_target_joint_pos[i].push_back(this->robot_joint_cmd_.joint_cmd[this->params.joint_mapping[i]].position);
        plt::subplot(this->params.num_of_dofs, 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.0001);
}

void RL_Real::UDPRecv()
{
    if (receiver_)
    {
        robot_data_ = &(receiver_->GetState());
    }
}

void RL_Real::EulerToQuaternion(float roll, float pitch, float yaw, float q[4])
{
    roll *= M_PI / 180.0f;
    pitch *= M_PI / 180.0f;
    yaw *= M_PI / 180.0f;

    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;  // w
    q[1] = sr * cp * cy - cr * sp * sy;  // x
    q[2] = cr * sp * cy + sr * cp * sy;  // y
    q[3] = cr * cp * sy - sr * sp * cy;  // z
}


#if !defined(USE_CMAKE) && defined(USE_ROS)
void RL_Real::CmdvelCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Real rl_sar;
    ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RL_Real>());
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    RL_Real rl_sar;
    while (1) { sleep(10); }
#endif
    return 0;
}
