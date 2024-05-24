#ifndef RL_REAL_HPP
#define RL_REAL_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "unitree_legged_sdk/loop.h"
#include <boost/bind.hpp>
#include <CustomInterface.h>
#include <csignal>
// #include <signal.h>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

using CyberdogData = Robot_Data;
using CyberdogCmd = Motor_Cmd;

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

    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_control;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_rl;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_plot;

    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<double>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

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