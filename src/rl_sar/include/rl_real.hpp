#ifndef RL_REAL_HPP
#define RL_REAL_HPP

#include "../library/rl/rl.hpp"
#include "../library/observation_buffer/observation_buffer.hpp"
#include <unitree_legged_msgs/LowCmd.h>
#include "unitree_legged_msgs/LowState.h"
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include <pthread.h>
#include <csignal>
// #include <signal.h>

using namespace UNITREE_LEGGED_SDK;

enum InitState {
    STATE_WAITING = 0,
    STATE_POS_INIT,
    STATE_RL_INIT,
    STATE_RL_START,
    STATE_POS_STOP,
};

class RL_Real : public RL
{
public:
    RL_Real();
    ~RL_Real();
    
    void runModel();
    torch::Tensor forward() override;
    torch::Tensor compute_observation() override;

    ObservationBuffer history_obs_buf;
    torch::Tensor history_obs;

    //udp
    void UDPSend();
    void UDPRecv();
    void RobotControl();
    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    xRockerBtnDataStruct _keyData;
    int motiontime = 0;

    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_udpSend;
    std::shared_ptr<LoopFunc> loop_udpRecv;
    std::shared_ptr<LoopFunc> loop_rl;

    float _percent;
	float _startPos[12];

    int init_state = STATE_WAITING;

private:
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;

    int dof_mapping[13] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};

    std::chrono::high_resolution_clock::time_point start_time;

    // other rl module
    torch::jit::script::Module encoder;
    torch::jit::script::Module vq;
};

#endif