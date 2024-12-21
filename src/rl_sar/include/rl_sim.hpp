#ifndef RL_SIM_HPP
#define RL_SIM_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>
#include "std_srvs/Empty.h"
#include <geometry_msgs/Twist.h>
#include "robot_msgs/MotorCommand.h"
#include <csignal>
#include <gazebo_msgs/SetModelState.h>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class RL_Sim : public RL
{
public:
    RL_Sim();
    ~RL_Sim();

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

    // ros interface
    std::string ros_namespace;
    geometry_msgs::Twist vel;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist cmd_vel;
    sensor_msgs::Joy joy_msg;
    ros::Subscriber model_state_subscriber;
    ros::Subscriber joint_state_subscriber;
    ros::Subscriber cmd_vel_subscriber;
    ros::Subscriber joy_subscriber;
    ros::ServiceClient gazebo_set_model_state_client;
    ros::ServiceClient gazebo_pause_physics_client;
    ros::ServiceClient gazebo_unpause_physics_client;
    std::map<std::string, ros::Publisher> joint_publishers;
    std::vector<robot_msgs::MotorCommand> joint_publishers_commands;
    void ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
    void JointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void JoyCallback(const sensor_msgs::Joy::ConstPtr &msg);

    // others
    std::string gazebo_model_name;
    int motiontime = 0;
    std::map<std::string, size_t> sorted_to_original_index;
    std::vector<double> mapped_joint_positions;
    std::vector<double> mapped_joint_velocities;
    std::vector<double> mapped_joint_efforts;
    void MapData(const std::vector<double> &source_data, std::vector<double> &target_data);
};

#endif // RL_SIM_HPP
