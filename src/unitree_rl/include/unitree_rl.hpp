#ifndef UNITREE_RL
#define UNITREE_RL

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "../lib/model.cpp"
#include "../lib/observation_buffer.hpp"
#include "unitree_legged_msgs/MotorCmd.h"

class Unitree_RL : public Model
{
public:
    Unitree_RL();
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void runModel(const ros::TimerEvent &event);
    torch::Tensor forward() override;
    torch::Tensor compute_observation() override;

    ObservationBuffer history_obs_buf;
    torch::Tensor history_obs;

private:
    std::string ros_namespace;

    std::vector<std::string> torque_command_topics;

    ros::Subscriber model_state_subscriber_;
    ros::Subscriber joint_state_subscriber_;
    ros::Subscriber cmd_vel_subscriber_;

    std::map<std::string, ros::Publisher> torque_publishers;
    std::vector<unitree_legged_msgs::MotorCmd> torque_commands;

    geometry_msgs::Twist vel;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist cmd_vel;

    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;

    torch::Tensor torques;

    ros::Timer timer;

    std::chrono::high_resolution_clock::time_point start_time;

    // other rl module
    torch::jit::script::Module encoder;
    torch::jit::script::Module vq;
};

#endif