#ifndef RL_REAL_HPP
#define RL_REAL_HPP

#include "../library/rl/rl.hpp"
#include "../library/observation_buffer/observation_buffer.hpp"
#include <unitree_legged_msgs/LowCmd.h>
#include "unitree_legged_msgs/LowState.h"
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <pthread.h>

using namespace UNITREE_LEGGED_SDK;

class RL_Real : public RL
{
public:
    RL_Real();
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

    std::vector<unitree_legged_msgs::MotorCmd> torque_commands;

    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;

    std::chrono::high_resolution_clock::time_point start_time;

    // other rl module
    torch::jit::script::Module encoder;
    torch::jit::script::Module vq;
};

#endif