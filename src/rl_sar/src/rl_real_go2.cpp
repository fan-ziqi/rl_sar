#include "rl_real_go2.hpp"

// #define PLOT
// #define CSV_LOGGER

RL_Real rl_sar;

void RL_Real::RL_Real()
{
    // read params from yaml
    this->robot_name = "go2_isaacgym";
    this->ReadYaml(this->robot_name);

    // history
    this->history_obs_buf = ObservationBuffer(1, this->params.num_observations, 6);

    // init robot
    this->InitRobotStateClient();
    while (rl_sar.QueryServiceStatus("sport_mode"))
    {
        std::cout << "Try to deactivate the service: " << "sport_mode" << std::endl;
        rl_sar.ActivateService("sport_mode", 0);
        sleep(1);
    }
    this->InitLowCmd();
    // create publisher
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();
    // create subscriber
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&RL_Real::LowStateMessageHandler, this, std::placeholders::_1), 1);
    // loop publishing thread TODO-devel-go2 why?
    // lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &RL_Real::LowCmdwriteHandler, this);

    // init rl
    torch::autograd::GradMode::set_enabled(false);
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();
    running_state = STATE_WAITING;

    // model
    std::string model_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + this->robot_name + "/" + this->params.model_name;
    this->model = torch::jit::load(model_path);

    // loop
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Real::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.num_of_dofs);
    this->plot_target_joint_pos.resize(this->params.num_of_dofs);
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real::~RL_Real()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    // TODO-devel-mutex 加锁
    memcpy(&this->unitree_joy, &this->unitree_low_state.wireless_remote()[0], 40);

    if ((int)this->unitree_joy.btn.components.R2 == 1)
    {
        this->control.control_state = STATE_POS_GETUP;
    }
    else if ((int)this->unitree_joy.btn.components.R1 == 1)
    {
        this->control.control_state = STATE_RL_INIT;
    }
    else if ((int)this->unitree_joy.btn.components.L2 == 1)
    {
        this->control.control_state = STATE_POS_GETDOWN;
    }

    if (this->params.framework == "isaacgym")
    {
        state->imu.quaternion[3] = this->unitree_low_state.imu_state().quaternion()[0]; // w
        state->imu.quaternion[0] = this->unitree_low_state.imu_state().quaternion()[1]; // x
        state->imu.quaternion[1] = this->unitree_low_state.imu_state().quaternion()[2]; // y
        state->imu.quaternion[2] = this->unitree_low_state.imu_state().quaternion()[3]; // z
    }
    else if (this->params.framework == "isaacsim")
    {
        state->imu.quaternion[0] = this->unitree_low_state.imu_state().quaternion()[0]; // w
        state->imu.quaternion[1] = this->unitree_low_state.imu_state().quaternion()[1]; // x
        state->imu.quaternion[2] = this->unitree_low_state.imu_state().quaternion()[2]; // y
        state->imu.quaternion[3] = this->unitree_low_state.imu_state().quaternion()[3]; // z
    }

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = this->unitree_low_state.imu_state().gyroscope()[i];
    }
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        state->motor_state.q[i] = this->unitree_low_state.motor_state()[state_mapping[i]].q();
        state->motor_state.dq[i] = this->unitree_low_state.motor_state()[state_mapping[i]].dq();
        state->motor_state.tauEst[i] = this->unitree_low_state.motor_state()[state_mapping[i]].tau_est();
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->unitree_low_command.motor_cmd()[i].mode() = 0x01;
        this->unitree_low_command.motor_cmd()[i].q() = command->motor_command.q[command_mapping[i]];
        this->unitree_low_command.motor_cmd()[i].dq() = command->motor_command.dq[command_mapping[i]];
        this->unitree_low_command.motor_cmd()[i].kp() = command->motor_command.kp[command_mapping[i]];
        this->unitree_low_command.motor_cmd()[i].kd() = command->motor_command.kd[command_mapping[i]];
        this->unitree_low_command.motor_cmd()[i].tau() = command->motor_command.tau[command_mapping[i]];
    }

    // 暂时不发 TODO-devel-go2 Why?
    this->unitree_low_command.crc() = Crc32Core((uint32_t *)&unitree_low_command, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(unitree_low_command);
}

void RL_Real::RobotControl()
{
    this->motiontime++;

    this->GetState(&this->robot_state);
    this->StateController(&this->robot_state, &this->robot_command);
    this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel()
{
    if (this->running_state == STATE_RL_RUNNING)
    {
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        // this->obs.commands = torch::tensor({{this->unitree_joy.ly, -this->unitree_joy.rx, -this->unitree_joy.lx}});
        this->obs.commands = torch::tensor({{this->control.x, this->control.y, this->control.yaw}});
        this->obs.base_quat = torch::tensor(this->robot_state.imu.quaternion).unsqueeze(0);
        this->obs.dof_pos = torch::tensor(this->robot_state.motor_state.q).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);
        this->obs.dof_vel = torch::tensor(this->robot_state.motor_state.dq).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);

        torch::Tensor clamped_actions = this->Forward();

        for (int i : this->params.hip_scale_reduction_indices)
        {
            clamped_actions[0][i] *= this->params.hip_scale_reduction;
        }

        this->obs.actions = clamped_actions;

        torch::Tensor origin_output_torques = this->ComputeTorques(this->obs.actions);

        this->TorqueProtect(origin_output_torques);

        this->output_torques = torch::clamp(origin_output_torques, -(this->params.torque_limits), this->params.torque_limits);
        this->output_dof_pos = this->ComputePosition(this->obs.actions);

#ifdef CSV_LOGGER
        torch::Tensor tau_est = torch::tensor(this->robot_state.motor_state.tauEst).unsqueeze(0);
        this->CSVLogger(this->output_torques, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

torch::Tensor RL_Real::Forward()
{
    torch::autograd::GradMode::set_enabled(false);

    torch::Tensor clamped_obs = this->ComputeObservation();

    this->history_obs_buf.insert(clamped_obs);
    this->history_obs = this->history_obs_buf.get_obs_vec({5, 4, 3, 2, 1, 0});

    torch::Tensor actions = this->model.forward({this->history_obs}).toTensor();

    if (this->params.clip_actions_upper.numel() != 0 && this->params.clip_actions_lower.numel() != 0)
    {
        return torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);
    }
    else
    {
        return actions;
    }
}

void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->unitree_low_state.motor_state()[i].q());
        this->plot_target_joint_pos[i].push_back(this->unitree_low_command.motor_cmd()[i].q());
        plt::subplot(4, 3, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.0001);
}

uint32_t RL_Real::Crc32Core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void RL_Real::InitLowCmd()
{
    unitree_low_command.head()[0] = 0xFE;
    unitree_low_command.head()[1] = 0xEF;
    unitree_low_command.level_flag() = 0xFF;
    unitree_low_command.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        unitree_low_command.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
        unitree_low_command.motor_cmd()[i].q() = (PosStopF);
        unitree_low_command.motor_cmd()[i].kp() = (0);
        unitree_low_command.motor_cmd()[i].dq() = (VelStopF);
        unitree_low_command.motor_cmd()[i].kd() = (0);
        unitree_low_command.motor_cmd()[i].tau() = (0);
    }
}

void RL_Real::InitRobotStateClient()
{
    rsc.SetTimeout(10.0f);
    rsc.Init();
}

int RL_Real::QueryServiceStatus(const std::string &serviceName)
{
    std::vector<ServiceState> serviceStateList;
    int ret, serviceStatus;
    ret = rsc.ServiceList(serviceStateList);
    size_t i, count = serviceStateList.size();
    for (i = 0; i < count; i++)
    {
        const ServiceState &serviceState = serviceStateList[i];
        if (serviceState.name == serviceName)
        {
            if (serviceState.status == 0)
            {
                std::cout << "name: " << serviceState.name << " is activate" << std::endl;
                serviceStatus = 1;
            }
            else
            {
                std::cout << "name:" << serviceState.name << " is deactivate" << std::endl;
                serviceStatus = 0;
            }
        }
    }
    return serviceStatus;
}

void RL_Real::ActivateService(const std::string &serviceName, int activate)
{
    rsc.ServiceSwitch(serviceName, activate);
}

void RL_Real::LowStateMessageHandler(const void *message)
{
    unitree_low_state = *(unitree_go::msg::dds_::LowState_ *)message;
}

void RL_Real::LowCmdwriteHandler()
{
    // this->unitree_low_command.crc() = Crc32Core((uint32_t *)&unitree_low_command, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    // this->lowcmd_publisher->Write(unitree_low_command);
}

void signalHandler(int signum)
{
    exit(0);
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);

    while (1)
    {
        sleep(10);
    };

    return 0;
}
