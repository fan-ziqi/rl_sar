#ifndef RL_REAL_HPP
#define RL_REAL_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/client/client.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include <csignal>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_JOYSTICK "rt/wirelesscontroller"
constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// 遥控器键值联合体
typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

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
    void InitLowCmd();
    int QueryServiceStatus(const std::string &serviceName);
    uint32_t Crc32Core(uint32_t *ptr, uint32_t len);
    void LowStateMessageHandler(const void *messages);
    void JoystickHandler(const void *message);
    RobotStateClient rsc;
    unitree_go::msg::dds_::LowCmd_ unitree_low_command{};
    unitree_go::msg::dds_::LowState_ unitree_low_state{};
    unitree_go::msg::dds_::WirelessController_ joystick{};
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_subscriber;
    xKeySwitchUnion unitree_joy;

    // others
    int motiontime = 0;
    std::vector<double> mapped_joint_positions;
    std::vector<double> mapped_joint_velocities;
    int command_mapping[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
    int state_mapping[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
};

#endif // RL_REAL_HPP
