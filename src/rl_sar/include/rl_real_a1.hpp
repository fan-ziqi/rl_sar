#ifndef RL_REAL_HPP
#define RL_REAL_HPP

// #include "../library/rl_sdk/rl_sdk.hpp"
#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include <unitree_legged_msgs/LowCmd.h>
#include "unitree_legged_msgs/LowState.h"
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include <csignal>
// #include <signal.h>

using namespace UNITREE_LEGGED_SDK;

enum RobotState {
    STATE_WAITING = 0,
    STATE_POS_GETUP,
    STATE_RL_INIT,
    STATE_RL_RUNNING,
    STATE_POS_GETDOWN,
};

class RL_Real : public RL
{
public:
    RL_Real();
    ~RL_Real();
    
    void RunModel();
    torch::Tensor Forward() override;
    torch::Tensor ComputeObservation() override;

    ObservationBuffer history_obs_buf;
    torch::Tensor history_obs;
    int motiontime = 0;

    //udp
    void UDPSend(){udp.Send();}
    void UDPRecv(){udp.Recv();}
    void RobotControl();
    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    xRockerBtnDataStruct _keyData;

    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_udpSend;
    std::shared_ptr<LoopFunc> loop_udpRecv;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    float getup_percent = 0.0;
    float getdown_percent = 0.0;
    float start_pos[12];
	float now_pos[12];

    int robot_state = STATE_WAITING;

    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<double>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();
private:
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;

    int dof_mapping[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
    int hip_scale_reduction_indices[4] = {0, 3, 6, 9};

    std::chrono::high_resolution_clock::time_point start_time;
};

#endif