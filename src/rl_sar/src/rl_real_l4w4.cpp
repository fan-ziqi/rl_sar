/*
* Copyright (c) 2024-2025 Ziqi Fan
* SPDX-License-Identifier: Apache-2.0
*/

#include "rl_real_l4w4.hpp"

RL_Real::RL_Real(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_node");
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
#endif

    // read params from yaml
    this->ang_vel_axis = "body";
    this->robot_name = "l4w4";
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->l4w4_sdk.InitUDP();
    this->l4w4_sdk.InitCmdData(this->l4w4_low_command);
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    // loop
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
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

void RL_Real::GetState(RobotState<float> *state)
{
    fd_set rfd;
    FD_ZERO(&rfd);
    FD_SET(this->l4w4_sdk.client_socket, &rfd);
    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    int ret = select(this->l4w4_sdk.client_socket + 1, &rfd, NULL, NULL, &timeout);

    if (ret > 0)
    {
        socklen_t addr_len = sizeof(struct sockaddr_in);
        this->l4w4_sdk.recv_len = recvfrom(this->l4w4_sdk.client_socket, this->l4w4_sdk.recv_buff, sizeof(this->l4w4_sdk.recv_buff), 0, (sockaddr *)&this->l4w4_sdk.server_addr, &addr_len);
        this->l4w4_sdk.ex_send_recv++;

        if (this->l4w4_sdk.recv_len < 32)
        {
            std::cout << " udp recv_len = " << this->l4w4_sdk.recv_len << std::endl;
            return;
        }
        this->l4w4_sdk.AnalyzeUDP(this->l4w4_sdk.recv_buff, this->l4w4_low_state);

        memcpy(&this->l4w4_joy, this->l4w4_low_state.wirelessRemote, 40);

        if (this->l4w4_joy.btn.components.A) this->control.SetGamepad(Input::Gamepad::A);
        if (this->l4w4_joy.btn.components.B) this->control.SetGamepad(Input::Gamepad::B);
        if (this->l4w4_joy.btn.components.X) this->control.SetGamepad(Input::Gamepad::X);
        if (this->l4w4_joy.btn.components.Y) this->control.SetGamepad(Input::Gamepad::Y);
        if (this->l4w4_joy.btn.components.L1) this->control.SetGamepad(Input::Gamepad::LB);
        if (this->l4w4_joy.btn.components.R1) this->control.SetGamepad(Input::Gamepad::RB);
        if (this->l4w4_joy.btn.components.F1) this->control.SetGamepad(Input::Gamepad::LStick);
        if (this->l4w4_joy.btn.components.F2) this->control.SetGamepad(Input::Gamepad::RStick);
        if (this->l4w4_joy.btn.components.up) this->control.SetGamepad(Input::Gamepad::DPadUp);
        if (this->l4w4_joy.btn.components.down) this->control.SetGamepad(Input::Gamepad::DPadDown);
        if (this->l4w4_joy.btn.components.left) this->control.SetGamepad(Input::Gamepad::DPadLeft);
        if (this->l4w4_joy.btn.components.right) this->control.SetGamepad(Input::Gamepad::DPadRight);
        if (this->l4w4_joy.btn.components.L1 && this->l4w4_joy.btn.components.A) this->control.SetGamepad(Input::Gamepad::LB_A);
        if (this->l4w4_joy.btn.components.L1 && this->l4w4_joy.btn.components.B) this->control.SetGamepad(Input::Gamepad::LB_B);
        if (this->l4w4_joy.btn.components.L1 && this->l4w4_joy.btn.components.X) this->control.SetGamepad(Input::Gamepad::LB_X);
        if (this->l4w4_joy.btn.components.L1 && this->l4w4_joy.btn.components.Y) this->control.SetGamepad(Input::Gamepad::LB_Y);
        if (this->l4w4_joy.btn.components.L1 && this->l4w4_joy.btn.components.F1) this->control.SetGamepad(Input::Gamepad::LB_LStick);
        if (this->l4w4_joy.btn.components.L1 && this->l4w4_joy.btn.components.F2) this->control.SetGamepad(Input::Gamepad::LB_RStick);
        if (this->l4w4_joy.btn.components.L1 && this->l4w4_joy.btn.components.up) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
        if (this->l4w4_joy.btn.components.L1 && this->l4w4_joy.btn.components.down) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
        if (this->l4w4_joy.btn.components.L1 && this->l4w4_joy.btn.components.left) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
        if (this->l4w4_joy.btn.components.L1 && this->l4w4_joy.btn.components.right) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
        if (this->l4w4_joy.btn.components.R1 && this->l4w4_joy.btn.components.A) this->control.SetGamepad(Input::Gamepad::RB_A);
        if (this->l4w4_joy.btn.components.R1 && this->l4w4_joy.btn.components.B) this->control.SetGamepad(Input::Gamepad::RB_B);
        if (this->l4w4_joy.btn.components.R1 && this->l4w4_joy.btn.components.X) this->control.SetGamepad(Input::Gamepad::RB_X);
        if (this->l4w4_joy.btn.components.R1 && this->l4w4_joy.btn.components.Y) this->control.SetGamepad(Input::Gamepad::RB_Y);
        if (this->l4w4_joy.btn.components.R1 && this->l4w4_joy.btn.components.F1) this->control.SetGamepad(Input::Gamepad::RB_LStick);
        if (this->l4w4_joy.btn.components.R1 && this->l4w4_joy.btn.components.F2) this->control.SetGamepad(Input::Gamepad::RB_RStick);
        if (this->l4w4_joy.btn.components.R1 && this->l4w4_joy.btn.components.up) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
        if (this->l4w4_joy.btn.components.R1 && this->l4w4_joy.btn.components.down) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
        if (this->l4w4_joy.btn.components.R1 && this->l4w4_joy.btn.components.left) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
        if (this->l4w4_joy.btn.components.R1 && this->l4w4_joy.btn.components.right) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
        if (this->l4w4_joy.btn.components.L1 && this->l4w4_joy.btn.components.R1) this->control.SetGamepad(Input::Gamepad::LB_RB);

        this->control.x = this->l4w4_joy.ly * 1.5f;
        this->control.y = -this->l4w4_joy.lx * 1.5f;
        this->control.yaw = -this->l4w4_joy.rx * 2.0f;

        state->imu.quaternion[0] = this->l4w4_low_state.imu.quaternion[0]; // w
        state->imu.quaternion[1] = this->l4w4_low_state.imu.quaternion[1]; // x
        state->imu.quaternion[2] = this->l4w4_low_state.imu.quaternion[2]; // y
        state->imu.quaternion[3] = this->l4w4_low_state.imu.quaternion[3]; // z

        for (int i = 0; i < 3; ++i)
        {
            state->imu.gyroscope[i] = this->l4w4_low_state.imu.gyroscope[i];
        }

        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            state->motor_state.q[i] = this->l4w4_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].q;
            state->motor_state.dq[i] = this->l4w4_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq;
            state->motor_state.tau_est[i] = this->l4w4_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].tauEst;
        }
    }
    else
    {
        sendto(this->l4w4_sdk.client_socket, this->l4w4_sdk.sent_buff, 96, 0, (const sockaddr *)&this->l4w4_sdk.server_addr, sizeof(this->l4w4_sdk.server_addr));
    }
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->l4w4_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].mode = 0x0A;
        this->l4w4_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].q = command->motor_command.q[i];
        this->l4w4_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq = command->motor_command.dq[i];
        this->l4w4_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].Kp = command->motor_command.kp[i];
        this->l4w4_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].Kd = command->motor_command.kd[i];
        this->l4w4_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau = command->motor_command.tau[i];
    }

    this->l4w4_sdk.SendUDP(this->l4w4_low_command);
}

void RL_Real::RobotControl()
{
    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#if !defined(USE_CMAKE) && defined(USE_ROS)
        if (this->control.navigation_mode)
        {
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};

        }
#endif
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
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
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->l4w4_low_state.motorState[i].q);
        this->plot_target_joint_pos[i].push_back(this->l4w4_low_command.motorCmd[i].q);
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.0001);
}

#if !defined(USE_CMAKE) && defined(USE_ROS)
void RL_Real::CmdvelCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Real rl_sar(argc, argv);
    ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Real>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    RL_Real rl_sar(argc, argv);
    while (1) { sleep(10); }
#endif
    return 0;
}
