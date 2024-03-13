#ifndef UNITREE_RL
#define UNITREE_RL

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "../lib/model.cpp"
#include "../lib/observation_buffer.hpp"
#include <unitree_legged_msgs/LowCmd.h>
#include "unitree_legged_msgs/LowState.h"
#include "convert.h"
#include <pthread.h>

using namespace UNITREE_LEGGED_SDK;

class Unitree_RL : public Model
{
public:
    Unitree_RL();
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void runModel();
    torch::Tensor forward() override;
    torch::Tensor compute_observation() override;

    ObservationBuffer history_obs_buf;
    torch::Tensor history_obs;

    torch::Tensor torques;

    //udp
    void UDPSend();
    void UDPRecv();
    void RobotControl();
    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int motiontime = 0;

    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_udpSend;
    std::shared_ptr<LoopFunc> loop_udpRecv;
    std::shared_ptr<LoopFunc> loop_rl;


    float _percent;
    float _targetPos[12] = {0.0, 0.8, -1.6, 0.0, 0.8, -1.6,
	                        0.0, 0.8, -1.6, 0.0, 0.8, -1.6}; //0.0, 0.67, -1.3
	float _startPos[12];

    bool init_done = false;

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

    

    ros::Timer timer;

    std::chrono::high_resolution_clock::time_point start_time;

    // other rl module
    torch::jit::script::Module encoder;
    torch::jit::script::Module vq;

    UNITREE_LEGGED_SDK::LowCmd SendLowLCM = {0};
    UNITREE_LEGGED_SDK::LowState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;
};

#endif