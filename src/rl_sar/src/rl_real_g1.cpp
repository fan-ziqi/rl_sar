/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_g1.hpp"

RL_Real::RL_Real()
#if defined(USE_ROS2)
    : rclcpp::Node("rl_real_node")
#endif
{
#if defined(USE_ROS1)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
#elif defined(USE_ROS2)
    this->cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
#endif

    // read params from yaml
    this->ang_vel_type = "ang_vel_body";
    this->robot_name = "g1";
    this->default_rl_config = "unitree_rl_gym";
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

    // init robot
    this->mode_pr = Mode::PR;
    this->mode_machine = 0;
    this->InitLowCmd();
    this->InitOutputs();
    this->InitControl();
    // create lowcmd publisher
    this->lowcmd_publisher.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    this->lowcmd_publisher->InitChannel();
    // create lowstate subscriber
    this->lowstate_subscriber.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    this->lowstate_subscriber->InitChannel(std::bind(&RL_Real::LowStateHandler, this, std::placeholders::_1), 1);
    // create imutorso subscriber
    this->imutorso_subscriber.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    this->imutorso_subscriber->InitChannel(std::bind(&RL_Real::ImuTorsoHandler, this, std::placeholders::_1), 1);
    // init MotionSwitcherClient
    this->msc.SetTimeout(5.0f);
    this->msc.Init();
    // Shut down motion control-related service
    std::string form, name;
    while (this->msc.CheckMode(form, name), !name.empty())
    {
        if (this->msc.ReleaseMode())
        {
            std::cout << "Failed to switch to Release Mode" << std::endl;
        }
        sleep(5);
    }

    // loop
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Real::RunModel, this));
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
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    if (this->unitree_low_state.crc() != Crc32Core((uint32_t *)&unitree_low_state, (sizeof(LowState_) >> 2) - 1))
    {
        std::cout << "[ERROR] CRC Error" << std::endl;
        return;
    }

    memcpy(this->remote_data_rx.buff, &unitree_low_state.wireless_remote()[0], 40);
    this->gamepad.update(this->remote_data_rx.RF_RX);

    this->control.x = this->gamepad.ly;
    this->control.y = -this->gamepad.lx;
    this->control.yaw = -this->gamepad.rx;

    if (this->mode_machine != this->unitree_low_state.mode_machine())
    {
        if (this->mode_machine == 0)
        {
            std::cout << "G1 type: " << unsigned(this->unitree_low_state.mode_machine()) << std::endl;
        }
        this->mode_machine = this->unitree_low_state.mode_machine();
    }

    if (this->gamepad.R2.pressed)
    {
        this->control.SetControlState(STATE_POS_GETUP);
    }
    else if (this->gamepad.R1.pressed)
    {
        this->control.SetControlState(STATE_RL_LOCOMOTION);
    }
    else if (this->gamepad.L2.pressed)
    {
        this->control.SetControlState(STATE_POS_GETDOWN);
    }
    else if (this->gamepad.down.pressed)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
    }

    state->imu.quaternion[0] = this->unitree_low_state.imu_state().quaternion()[0]; // w
    state->imu.quaternion[1] = this->unitree_low_state.imu_state().quaternion()[1]; // x
    state->imu.quaternion[2] = this->unitree_low_state.imu_state().quaternion()[2]; // y
    state->imu.quaternion[3] = this->unitree_low_state.imu_state().quaternion()[3]; // z

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = this->unitree_low_state.imu_state().gyroscope()[i];
    }
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        state->motor_state.q[i] = this->unitree_low_state.motor_state()[this->params.state_mapping[i]].q();
        state->motor_state.dq[i] = this->unitree_low_state.motor_state()[this->params.state_mapping[i]].dq();
        state->motor_state.tau_est[i] = this->unitree_low_state.motor_state()[this->params.state_mapping[i]].tau_est();
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    this->unitree_low_command.mode_pr() = static_cast<uint8_t>(this->mode_pr);
    this->unitree_low_command.mode_machine() = this->mode_machine;

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->unitree_low_command.motor_cmd()[i].mode() = 1; // 1:Enable, 0:Disable
        this->unitree_low_command.motor_cmd()[i].q() = command->motor_command.q[this->params.command_mapping[i]];
        this->unitree_low_command.motor_cmd()[i].dq() = command->motor_command.dq[this->params.command_mapping[i]];
        this->unitree_low_command.motor_cmd()[i].kp() = command->motor_command.kp[this->params.command_mapping[i]];
        this->unitree_low_command.motor_cmd()[i].kd() = command->motor_command.kd[this->params.command_mapping[i]];
        this->unitree_low_command.motor_cmd()[i].tau() = command->motor_command.tau[this->params.command_mapping[i]];
    }

    this->unitree_low_command.crc() = Crc32Core((uint32_t *)&unitree_low_command, (sizeof(LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(unitree_low_command);
}

void RL_Real::RobotControl()
{
    this->motiontime++;

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
#if !defined(USE_CMAKE)
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
        this->plot_real_joint_pos[i].push_back(this->unitree_low_state.motor_state()[i].q());
        this->plot_target_joint_pos[i].push_back(this->unitree_low_command.motor_cmd()[i].q());
        plt::subplot(this->params.num_of_dofs, 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.0001);
}

uint32_t RL_Real::Crc32Core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; ++i)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
            {
                CRC32 ^= dwPolynomial;
            }
            xbit >>= 1;
        }
    }

    return CRC32;
}

void RL_Real::InitLowCmd()
{
    for (int i = 0; i < 32; ++i)
    {
        this->unitree_low_command.motor_cmd()[i].mode() = (1); // 1:Enable, 0:Disable
        this->unitree_low_command.motor_cmd()[i].q() = (0);
        this->unitree_low_command.motor_cmd()[i].kp() = (0);
        this->unitree_low_command.motor_cmd()[i].dq() = (0);
        this->unitree_low_command.motor_cmd()[i].kd() = (0);
        this->unitree_low_command.motor_cmd()[i].tau() = (0);
    }
}

void RL_Real::LowStateHandler(const void *message)
{
    this->unitree_low_state = *(LowState_ *)message;
}

void RL_Real::ImuTorsoHandler(const void *message)
{
    this->unitree_imu_torso = *(const IMUState_ *)message;
}

#if !defined(USE_CMAKE)
void RL_Real::CmdvelCallback(
#if defined(USE_ROS1)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}
#endif

#if defined(USE_ROS1)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface [wheel]" << std::endl;
        exit(-1);
    }
    ChannelFactory::Instance()->Init(0, argv[1]);
#if defined(USE_ROS1)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Real rl_sar;
    ros::spin();
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RL_Real>());
    rclcpp::shutdown();
#elif defined(USE_CMAKE)
    RL_Real rl_sar;
    while (1) { sleep(10); }
#endif
    return 0;
}
