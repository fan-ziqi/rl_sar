#include "../include/rl_real_a1.hpp"

// #define PLOT
// #define CSV_LOGGER

RL_Real rl_sar;

RL_Real::RL_Real() : unitree_safe(UNITREE_LEGGED_SDK::LeggedType::A1), unitree_udp(UNITREE_LEGGED_SDK::LOWLEVEL)
{
    // read params from yaml
    robot_name = "a1";
    ReadYaml(robot_name);

    // history
    this->history_obs_buf = ObservationBuffer(1, this->params.num_observations, 6);

    unitree_udp.InitCmdData(unitree_low_command);

    // init
    torch::autograd::GradMode::set_enabled(false);
    start_pos.resize(params.num_of_dofs);
    now_pos.resize(params.num_of_dofs);
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();

    // model
    std::string model_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + robot_name + "/" + this->params.model_name;
    this->model = torch::jit::load(model_path);

    // loop
    loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05 ,    boost::bind(&RL_Real::KeyboardInterface, this));
    loop_control  = std::make_shared<LoopFunc>("loop_control" , 0.002,    boost::bind(&RL_Real::RobotControl     , this));
    loop_udpSend  = std::make_shared<LoopFunc>("loop_udpSend" , 0.002, 3, boost::bind(&RL_Real::UDPSend          , this));
    loop_udpRecv  = std::make_shared<LoopFunc>("loop_udpRecv" , 0.002, 3, boost::bind(&RL_Real::UDPRecv          , this));
    loop_rl       = std::make_shared<LoopFunc>("loop_rl"      , 0.02 ,    boost::bind(&RL_Real::RunModel         , this));
    loop_keyboard->start();
    loop_udpSend->start();
    loop_udpRecv->start();
    loop_control->start();
    loop_rl->start();

#ifdef PLOT
    plot_t = std::vector<int>(plot_size, 0);
    plot_real_joint_pos.resize(params.num_of_dofs);
    plot_target_joint_pos.resize(params.num_of_dofs);
    for(auto& vector : plot_real_joint_pos) { vector = std::vector<double>(plot_size, 0); }
    for(auto& vector : plot_target_joint_pos) { vector = std::vector<double>(plot_size, 0); }
    loop_plot    = std::make_shared<LoopFunc>("loop_plot"   , 0.002,    boost::bind(&RL_Real::Plot,         this));
    loop_plot->start();
#endif
#ifdef CSV_LOGGER
    CSVInit(robot_name);
#endif
}

RL_Real::~RL_Real()
{
    loop_keyboard->shutdown();
    loop_udpSend->shutdown();
    loop_udpRecv->shutdown();
    loop_control->shutdown();
    loop_rl->shutdown();
#ifdef PLOT
    loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    unitree_udp.GetRecv(unitree_low_state);
    memcpy(&unitree_joy, unitree_low_state.wirelessRemote, 40);

    if((int)unitree_joy.btn.components.R2 == 1)
    {
        control.control_state = STATE_POS_GETUP;
    }
    else if((int)unitree_joy.btn.components.R1 == 1)
    {
        control.control_state = STATE_RL_INIT;
    }
    else if((int)unitree_joy.btn.components.L2 == 1)
    {
        control.control_state = STATE_POS_GETDOWN;
    }

    state->imu.quaternion[3] = unitree_low_state.imu.quaternion[0]; // w
    state->imu.quaternion[0] = unitree_low_state.imu.quaternion[1]; // x
    state->imu.quaternion[1] = unitree_low_state.imu.quaternion[2]; // y
    state->imu.quaternion[2] = unitree_low_state.imu.quaternion[3]; // z
    for(int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = unitree_low_state.imu.gyroscope[i];
    }
    for(int i = 0; i < params.num_of_dofs; ++i)
    {
        state->motor_state.q[i] = unitree_low_state.motorState[state_mapping[i]].q;
        state->motor_state.dq[i] = unitree_low_state.motorState[state_mapping[i]].dq;
        state->motor_state.tauEst[i] = unitree_low_state.motorState[state_mapping[i]].tauEst;
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    for(int i = 0; i < params.num_of_dofs; ++i)
    {
        unitree_low_command.motorCmd[i].mode = 0x0A;
        unitree_low_command.motorCmd[i].q = command->motor_command.q[command_mapping[i]];
        unitree_low_command.motorCmd[i].dq = command->motor_command.dq[command_mapping[i]];
        unitree_low_command.motorCmd[i].Kp = command->motor_command.kp[command_mapping[i]];
        unitree_low_command.motorCmd[i].Kd = command->motor_command.kd[command_mapping[i]];
        unitree_low_command.motorCmd[i].tau = command->motor_command.tau[command_mapping[i]];
    }

    unitree_safe.PowerProtect(unitree_low_command, unitree_low_state, 6);
    // unitree_safe.PositionProtect(unitree_low_command, unitree_low_state);
    unitree_udp.SetSend(unitree_low_command);
}

void RL_Real::RobotControl()
{
    motiontime++;

    GetState(&robot_state);
    StateController(&robot_state, &robot_command);
    SetCommand(&robot_command);
}

void RL_Real::RunModel()
{
    if(running_state == STATE_RL_RUNNING)
    {
        this->obs.ang_vel = torch::tensor(robot_state.imu.gyroscope).unsqueeze(0);
        this->obs.commands = torch::tensor({{unitree_joy.ly, -unitree_joy.rx, -unitree_joy.lx}});
        this->obs.base_quat = torch::tensor(robot_state.imu.quaternion).unsqueeze(0);
        this->obs.dof_pos = torch::tensor(robot_state.motor_state.q).narrow(0, 0, params.num_of_dofs).unsqueeze(0);
        this->obs.dof_vel = torch::tensor(robot_state.motor_state.dq).narrow(0, 0, params.num_of_dofs).unsqueeze(0);

        torch::Tensor clamped_actions = this->Forward();

        for (int i : this->params.hip_scale_reduction_indices)
        {
            clamped_actions[0][i] *= this->params.hip_scale_reduction;
        }

        this->obs.actions = clamped_actions;

        torch::Tensor origin_output_torques = this->ComputeTorques(this->obs.actions);

        TorqueProtect(origin_output_torques);

        output_torques = torch::clamp(origin_output_torques, -(this->params.torque_limits), this->params.torque_limits);
        output_dof_pos = this->ComputePosition(this->obs.actions);

#ifdef CSV_LOGGER
        torch::Tensor tau_est = torch::tensor(robot_state.motor_state.tauEst).unsqueeze(0);
        CSVLogger(output_torques, tau_est, this->obs.dof_pos, output_dof_pos, this->obs.dof_vel);
#endif
    }
}

torch::Tensor RL_Real::ComputeObservation()
{
    torch::Tensor obs = torch::cat({// this->QuatRotateInverse(this->obs.base_quat, this->obs.lin_vel) * this->params.lin_vel_scale,
                                    this->QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) * this->params.ang_vel_scale,
                                    this->QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec),
                                    this->obs.commands * this->params.commands_scale,
                                    (this->obs.dof_pos - this->params.default_dof_pos) * this->params.dof_pos_scale,
                                    this->obs.dof_vel * this->params.dof_vel_scale,
                                    this->obs.actions
                                    },1);
    torch::Tensor clamped_obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);
    return clamped_obs;
}

torch::Tensor RL_Real::Forward()
{
    torch::autograd::GradMode::set_enabled(false);

    torch::Tensor clamped_obs = this->ComputeObservation();

    history_obs_buf.insert(clamped_obs);
    history_obs = history_obs_buf.get_obs_vec({0, 1, 2, 3, 4, 5});

    torch::Tensor actions = this->model.forward({history_obs}).toTensor();

    torch::Tensor clamped_actions = torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);

    return clamped_actions;
}

void RL_Real::Plot()
{
    plot_t.erase(plot_t.begin());
    plot_t.push_back(motiontime);
    plt::cla();
    plt::clf();
    for(int i = 0; i < params.num_of_dofs; ++i)
    {
        plot_real_joint_pos[i].erase(plot_real_joint_pos[i].begin());
        plot_target_joint_pos[i].erase(plot_target_joint_pos[i].begin());
        plot_real_joint_pos[i].push_back(unitree_low_state.motorState[i].q);
        plot_target_joint_pos[i].push_back(unitree_low_command.motorCmd[i].q);
        plt::subplot(4, 3, i + 1);
        plt::named_plot("_real_joint_pos", plot_t, plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", plot_t, plot_target_joint_pos[i], "b");
        plt::xlim(plot_t.front(), plot_t.back());
    }
    // plt::legend();
    plt::pause(0.0001);
}

void signalHandler(int signum)
{
    exit(0);
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);

    while(1)
    {
        sleep(10);
    };

    return 0;
}
