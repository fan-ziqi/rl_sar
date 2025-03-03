/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_L4W4_HPP
#define RL_REAL_L4W4_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "l4w4_sdk.hpp"
#include <csignal>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class RL_Real : public RL
{
public:
    RL_Real();
    ~RL_Real();

private:
    // rl functions
    torch::Tensor Forward() override;
    void GetState(RobotState<double> *state) override;
    void SetCommand(const RobotCommand<double> *command) override;
    void RunModel();
    void RobotControl();

    // history buffer
    ObservationBuffer history_obs_buf;
    torch::Tensor history_obs;

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<double>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // l4w4 interface
    L4W4SDK l4w4_sdk;
    LowCmd l4w4_low_command = {0};
    LowState l4w4_low_state = {0};
    xRockerBtnDataStruct unitree_joy;

    // others
    int motiontime = 0;
    std::vector<double> mapped_joint_positions;
    std::vector<double> mapped_joint_velocities;
};

#endif // RL_REAL_L4W4_HPP
