#ifndef RL_REAL_HPP
#define RL_REAL_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include <unitree/robot/channel/channel_publisher.hpp> // TODO-devel-go2 go2的sdk没有传上来
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include "unitree_joystick.h" // TODO-devel-go2 这里调用的是a1的，go2有自己的键盘接口吗
#include <csignal>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class RL_Real : public RL
{
public:
    RL_Real();
    ~RL_Real();

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

    // unitree interface
    void InitRobotStateClient();
    int QueryServiceStatus(const std::string &serviceName);
    void ActivateService(const std::string &serviceName, int activate);
    void LowCmdwriteHandler();
    uint32_t Crc32Core(uint32_t *ptr, uint32_t len);
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    RobotStateClient rsc;
    unitree_go::msg::dds_::LowCmd_ unitree_low_command{}; // default init
    unitree_go::msg::dds_::LowState_ unitree_low_state{}; // default init
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher; // publisher
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber; //subscriber
    xRockerBtnDataStruct unitree_joy;

    // others
    int motiontime = 0;
    std::vector<double> mapped_joint_positions;
    std::vector<double> mapped_joint_velocities;
    int command_mapping[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
    int state_mapping[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
};

#endif // RL_REAL_HPP
