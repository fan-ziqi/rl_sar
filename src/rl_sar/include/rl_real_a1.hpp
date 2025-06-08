/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_A1_HPP
#define RL_REAL_A1_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
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
    std::shared_ptr<LoopFunc> loop_udpSend;
    std::shared_ptr<LoopFunc> loop_udpRecv;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<double>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // unitree interface
    void UDPSend() { unitree_udp.Send(); }
    void UDPRecv() { unitree_udp.Recv(); }
    UNITREE_LEGGED_SDK::Safety unitree_safe;
    UNITREE_LEGGED_SDK::UDP unitree_udp;
    UNITREE_LEGGED_SDK::LowCmd unitree_low_command = {0};
    UNITREE_LEGGED_SDK::LowState unitree_low_state = {0};
    xRockerBtnDataStruct unitree_joy;

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

#endif // RL_REAL_A1_HPP
