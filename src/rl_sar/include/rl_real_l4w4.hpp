/*
* Copyright (c) 2024-2025 Ziqi Fan
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef RL_REAL_L4W4_HPP
#define RL_REAL_L4W4_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "l4w4_sdk.hpp"
#include <csignal>

#ifdef USE_ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#endif

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
    xRockerBtnDataStruct l4w4_joy;

    // others
    int motiontime = 0;
    std::vector<double> mapped_joint_positions;
    std::vector<double> mapped_joint_velocities;

#ifdef USE_ROS
    // ros
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
#endif
};

#endif // RL_REAL_L4W4_HPP
