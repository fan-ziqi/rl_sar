#ifndef RL_SIM_HPP
#define RL_SIM_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_sdk/loop.h"
#include <csignal>

// #include "robot_msgs/MotorCommand.h"
// #include "robot_msgs/RobotState.h"
// #include "robot_msgs/RobotCommand.h"

class RL_Sim : public RL
{
public:
    RL_Sim();
    ~RL_Sim();

    void ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
    void JointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    
    void RunModel();
    void RobotControl();
    torch::Tensor Forward() override;
    torch::Tensor ComputeObservation() override;

    ObservationBuffer history_obs_buf;
    torch::Tensor history_obs;

    int motiontime = 0;

    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_control;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_rl;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_plot;

    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<double>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();
private:
    std::string ros_namespace;

    std::vector<std::string> torque_command_topics;

    ros::Subscriber model_state_subscriber_;
    ros::Subscriber joint_state_subscriber_;
    ros::Subscriber cmd_vel_subscriber_;

    std::map<std::string, ros::Publisher> torque_publishers;
    std::vector<unitree_legged_msgs::MotorCmd> motor_commands;

    // std::vector<robot_msgs::MotorCommand> motor_commands;
    // robot_msgs::RobotState robot_state;
    // robot_msgs::RobotCommand robot_command;

    geometry_msgs::Twist vel;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist cmd_vel;

    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;

    std::chrono::high_resolution_clock::time_point start_time;

    void MapData(const std::vector<double>& source_data, std::vector<double>& target_data);
    std::map<std::string, size_t> sorted_to_original_index;
};

#endif