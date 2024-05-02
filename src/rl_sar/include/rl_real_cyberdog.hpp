#ifndef RL_REAL_HPP
#define RL_REAL_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "unitree_legged_sdk/loop.h"
#include <boost/bind.hpp>
#include <CustomInterface.h>
#include <csignal>
// #include <signal.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <iostream>

using namespace UNITREE_LEGGED_SDK;

using CyberdogData = Robot_Data;
using CyberdogCmd = Motor_Cmd;

enum RobotState {
    STATE_WAITING = 0,
    STATE_POS_GETUP,
    STATE_RL_INIT,
    STATE_RL_RUNNING,
    STATE_POS_GETDOWN,
};

struct KeyBoard
{
    RobotState robot_state;
    float x = 0;
    float y = 0;
    float yaw = 0;
};

class RL_Real : public RL, public CustomInterface
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

    CyberdogData cyberdogData;
	CyberdogCmd cyberdogCmd;
    void UserCode();
	long long count = 0;
    
    void RobotControl();

    std::shared_ptr<LoopFunc> loop_control;
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

    void run_keyboard();
    KeyBoard keyboard;
    std::thread _keyboardThread;
private:
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;

    int dof_mapping[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
    int hip_scale_reduction_indices[4] = {0, 3, 6, 9};

    std::chrono::high_resolution_clock::time_point start_time;
};

#endif