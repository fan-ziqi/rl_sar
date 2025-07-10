/*
 * Copyright (c) 2024-2025 Yuxuan Ma
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_LITE3_HPP
#define RL_REAL_LITE3_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm.hpp"

// Lite3 SDK
#include "sender.h"
#include "receiver.h"
#include "robot_types.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>



class RL_Real : public RL
#if defined(USE_ROS2) && defined(USE_ROS)
    , public rclcpp::Node
#endif
{
public:
    RL_Real();
    ~RL_Real();

private:
    // RL interface
    torch::Tensor Forward() override;
    void GetState(RobotState<double>* state) override;
    void SetCommand(const RobotCommand<double>* command) override;
    void RunModel();
    void RobotControl();

    // Loop functions
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_udpSend;
    std::shared_ptr<LoopFunc> loop_udpRecv;
    std::shared_ptr<LoopFunc> loop_rl;

    // Lite3 SDK interface
    Sender* sender_ = nullptr;
    Receiver* receiver_ = nullptr;
    RobotCmd robot_joint_cmd_{};
    RobotData* robot_data_=nullptr;
    // void UDPSend();
    void UDPRecv();
    void EulerToQuaternion(float roll, float pitch, float yaw, float q[4]);

    // others
    int motiontime = 0;
    std::vector<double> mapped_joint_positions;
    std::vector<double> mapped_joint_velocities;

#if defined(USE_ROS1) && defined(USE_ROS)
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg);
#elif defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
#endif
};

#endif  // RL_REAL_LITE3_HPP