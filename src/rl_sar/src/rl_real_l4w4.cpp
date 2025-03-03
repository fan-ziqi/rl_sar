/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_l4w4.hpp"

// #define PLOT
// #define CSV_LOGGER

RL_Real::RL_Real()
{
    // read params from yaml
    this->robot_name = "l4w4";
    this->config_name = "legged_gym";
    std::string robot_path = this->robot_name + "/" + this->config_name;
    this->ReadYaml(robot_path);
    for (std::string &observation : this->params.observations)
    {
        // In Unitree A1, the coordinate system for angular velocity is in the body coordinate system.
        if (observation == "ang_vel")
        {
            observation = "ang_vel_body";
        }
    }

    // init robot
    this->l4w4_sdk.InitUDP();
    this->l4w4_sdk.InitCmdData(this->l4w4_low_command);

    // init rl
    torch::autograd::GradMode::set_enabled(false);
    torch::set_num_threads(4);
    if (!this->params.observations_history.empty())
    {
        this->history_obs_buf = ObservationBuffer(1, this->params.num_observations, this->params.observations_history.size());
    }
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();
    running_state = STATE_WAITING;

    // model
    std::string model_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + robot_path + "/" + this->params.model_name;
    this->model = torch::jit::load(model_path);

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
    fd_set rfd;
    FD_ZERO(&rfd);
    FD_SET(this->l4w4_sdk.client_socket, &rfd);
    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    int ret = select(this->l4w4_sdk.client_socket+1, &rfd, NULL, NULL, &timeout);

    if(ret > 0)
    {
        socklen_t addr_len = sizeof(struct sockaddr_in);
        this->l4w4_sdk.recv_len = recvfrom(this->l4w4_sdk.client_socket, this->l4w4_sdk.recv_buff, sizeof(this->l4w4_sdk.recv_buff), 0, (sockaddr*)& this->l4w4_sdk.server_addr, & addr_len);
        this->l4w4_sdk.ex_send_recv++;

        if(this->l4w4_sdk.recv_len < 32)
        {
            std::cout<<" udp recv_len = "<<this->l4w4_sdk.recv_len<<std::endl;
            return;
        }
        this->l4w4_sdk.AnalyzeUDP(this->l4w4_sdk.recv_buff, this->l4w4_low_state);

        memcpy(&this->unitree_joy, this->l4w4_low_state.wirelessRemote, 40);

        if ((int)this->unitree_joy.btn.components.R2 == 1)
        {
            this->control.control_state = STATE_POS_GETUP;
        }
        else if ((int)this->unitree_joy.btn.components.R1 == 1)
        {
            this->control.control_state = STATE_RL_INIT;
        }
        else if ((int)this->unitree_joy.btn.components.L2 == 1)
        {
            this->control.control_state = STATE_POS_GETDOWN;
        }

        if (this->params.framework == "isaacgym")
        {
            state->imu.quaternion[3] = this->l4w4_low_state.imu.quaternion[0]; // w
            state->imu.quaternion[0] = this->l4w4_low_state.imu.quaternion[1]; // x
            state->imu.quaternion[1] = this->l4w4_low_state.imu.quaternion[2]; // y
            state->imu.quaternion[2] = this->l4w4_low_state.imu.quaternion[3]; // z
        }
        else if (this->params.framework == "isaacsim")
        {
            state->imu.quaternion[0] = this->l4w4_low_state.imu.quaternion[0]; // w
            state->imu.quaternion[1] = this->l4w4_low_state.imu.quaternion[1]; // x
            state->imu.quaternion[2] = this->l4w4_low_state.imu.quaternion[2]; // y
            state->imu.quaternion[3] = this->l4w4_low_state.imu.quaternion[3]; // z
        }


        for (int i = 0; i < 3; ++i)
        {
            state->imu.gyroscope[i] = this->l4w4_low_state.imu.gyroscope[i];
        }

        for (int i = 0; i < this->params.num_of_dofs; ++i)
        {
            state->motor_state.q[i] = this->l4w4_low_state.motorState[this->params.state_mapping[i]].q;
            state->motor_state.dq[i] = this->l4w4_low_state.motorState[this->params.state_mapping[i]].dq;
            state->motor_state.tau_est[i] = this->l4w4_low_state.motorState[this->params.state_mapping[i]].tauEst;
        }
    }
    else
    {
        sendto(this->l4w4_sdk.client_socket, this->l4w4_sdk.sent_buff, 96, 0, (const sockaddr*)& this->l4w4_sdk.server_addr, sizeof(this->l4w4_sdk.server_addr));
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->l4w4_low_command.motorCmd[i].mode = 0x0A;
        this->l4w4_low_command.motorCmd[i].q = command->motor_command.q[this->params.command_mapping[i]];
        this->l4w4_low_command.motorCmd[i].dq = command->motor_command.dq[this->params.command_mapping[i]];
        this->l4w4_low_command.motorCmd[i].Kp = command->motor_command.kp[this->params.command_mapping[i]];
        this->l4w4_low_command.motorCmd[i].Kd = command->motor_command.kd[this->params.command_mapping[i]];
        this->l4w4_low_command.motorCmd[i].tau = command->motor_command.tau[this->params.command_mapping[i]];
    }

    this->l4w4_sdk.SendUDP(this->l4w4_low_command);
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
    // this->l4w4_sdk.PrintMCU(this->running_state);

    if (this->running_state == STATE_RL_RUNNING)
    {
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        this->obs.commands = torch::tensor({{this->unitree_joy.ly * 1.5f, -this->unitree_joy.lx * 1.5f, -this->unitree_joy.rx * 2}});
        // this->obs.commands = torch::tensor({{this->control.x, this->control.y, this->control.yaw}});
        this->obs.base_quat = torch::tensor(this->robot_state.imu.quaternion).unsqueeze(0);
        this->obs.dof_pos = torch::tensor(this->robot_state.motor_state.q).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);
        this->obs.dof_vel = torch::tensor(this->robot_state.motor_state.dq).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);

        torch::Tensor clamped_actions = this->Forward();

        this->obs.actions = clamped_actions;

        for (int i : this->params.hip_scale_reduction_indices)
        {
            clamped_actions[0][i] *= this->params.hip_scale_reduction;
        }

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

        this->TorqueProtect(this->output_dof_tau);
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
        this->plot_real_joint_pos[i].push_back(this->l4w4_low_state.motorState[i].q);
        this->plot_target_joint_pos[i].push_back(this->l4w4_low_command.motorCmd[i].q);
        plt::subplot(4, 3, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.0001);
}

void signalHandler(int signum)
{
    exit(0);
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);

    RL_Real rl_sar;

    while (1)
    {
        sleep(10);
    }

    return 0;
}
