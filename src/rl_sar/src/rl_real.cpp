#include "../include/rl_real.hpp"

#define PLOT

RL_Real rl_sar;

void RL_Real::RobotControl()
{
    motiontime++;
    udp.GetRecv(state);

    memcpy(&_keyData, state.wirelessRemote, 40);

    // waiting
    if(robot_state == STATE_WAITING)
    {
        for(int i = 0; i < 12; ++i)
        {
            cmd.motorCmd[i].q = state.motorState[i].q;
        }
        if((int)_keyData.btn.components.R2 == 1)
        {
            getup_percent = 0.0;
            for(int i = 0; i < 12; ++i)
            {
                now_pos[i] = state.motorState[i].q;
                start_pos[i] = now_pos[i];
            }
            robot_state = STATE_POS_GETUP;
        }
    }
    // stand up (position control)
    else if(robot_state == STATE_POS_GETUP)
    {
        if(getup_percent != 1)
        {
            getup_percent += 1 / 1000.0;
            getup_percent = getup_percent > 1 ? 1 : getup_percent;
            for(int i = 0; i < 12; ++i)
            {
                cmd.motorCmd[i].mode = 0x0A;
                cmd.motorCmd[i].q = (1 - getup_percent) * now_pos[i] + getup_percent * params.default_dof_pos[0][dof_mapping[i]].item<double>();
                cmd.motorCmd[i].dq = 0;
                cmd.motorCmd[i].Kp = 50;
                cmd.motorCmd[i].Kd = 3;
                cmd.motorCmd[i].tau = 0;
            }
            printf("getting up %.3f%%\r", getup_percent*100.0);
        }
        if((int)_keyData.btn.components.R1 == 1)
        {
            robot_state = STATE_RL_INIT;
        }
        else if((int)_keyData.btn.components.L2 == 1)
        {
            getdown_percent = 0.0;
            for(int i = 0; i < 12; ++i)
            {
                now_pos[i] = state.motorState[i].q;
            }
            robot_state = STATE_POS_GETDOWN;
        }
    }
    // init obs and start rl loop
    else if(robot_state == STATE_RL_INIT)
    {
        if(getup_percent == 1)
        {
            robot_state = STATE_RL_RUNNING;
            this->InitObservations();
            printf("\nstart rl loop\n");
            loop_rl->start();
        }
    }
    // rl loop
    else if(robot_state == STATE_RL_RUNNING)
    {
        for(int i = 0; i < 12; ++i)
        {
            cmd.motorCmd[i].mode = 0x0A;
            cmd.motorCmd[i].q = output_dof_pos[0][dof_mapping[i]].item<double>();
            cmd.motorCmd[i].dq = 0;
            // cmd.motorCmd[i].Kp = Kp[dof_mapping[i]];
            // cmd.motorCmd[i].Kd = Kd[dof_mapping[i]];
            cmd.motorCmd[i].Kp = params.stiffness;
            cmd.motorCmd[i].Kd = params.damping;
            // cmd.motorCmd[i].tau = output_torques[0][dof_mapping[i]].item<double>();
            cmd.motorCmd[i].tau = 0;
        }
        if((int)_keyData.btn.components.L2 == 1)
        {
            getdown_percent = 0.0;
            for(int i = 0; i < 12; ++i)
            {
                now_pos[i] = state.motorState[i].q;
            }
            robot_state = STATE_POS_GETDOWN;
        }
    }
    // get down (position control)
    else if(robot_state == STATE_POS_GETDOWN)
    {
        if(getdown_percent != 1)
        {
            getdown_percent += 1 / 1000.0;
            getdown_percent = getdown_percent > 1 ? 1 : getdown_percent;
            for(int i = 0; i < 12; ++i)
            {
                cmd.motorCmd[i].mode = 0x0A;
                cmd.motorCmd[i].q = (1 - getdown_percent) * now_pos[i] + getdown_percent * start_pos[i];
                cmd.motorCmd[i].dq = 0;
                cmd.motorCmd[i].Kp = 50;
                cmd.motorCmd[i].Kd = 3;
                cmd.motorCmd[i].tau = 0;
            }
            printf("getting down %.3f%%\r", getdown_percent*100.0);
        }
        if(getdown_percent == 1)
        {
            robot_state = STATE_WAITING;
            this->InitObservations();
            printf("\nstop rl loop\n");
            loop_rl->shutdown();
        }
    }

    safe.PowerProtect(cmd, state, 7);
    udp.SetSend(cmd);
}

RL_Real::RL_Real() : safe(LeggedType::A1), udp(LOWLEVEL)
{
    udp.InitCmdData(cmd);

    start_time = std::chrono::high_resolution_clock::now();

    std::string actor_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/actor.pt";
    std::string encoder_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/encoder.pt";
    std::string vq_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/vq_layer.pt";

    this->actor = torch::jit::load(actor_path);
    this->encoder = torch::jit::load(encoder_path);
    this->vq = torch::jit::load(vq_path);
    this->InitObservations();
    
    this->params.num_observations = 45;
    this->params.clip_obs = 100.0;
    this->params.clip_actions = 100.0;
    this->params.damping = 0.5;
    this->params.stiffness = 20;
    this->params.d_gains = torch::ones(12) * this->params.damping;
    this->params.p_gains = torch::ones(12) * this->params.stiffness;
    this->params.action_scale = 0.25;
    this->params.hip_scale_reduction = 0.5;
    this->params.num_of_dofs = 12;
    this->params.lin_vel_scale = 2.0;
    this->params.ang_vel_scale = 0.25;
    this->params.dof_pos_scale = 1.0;
    this->params.dof_vel_scale = 0.05;
    this->params.commands_scale = torch::tensor({this->params.lin_vel_scale, this->params.lin_vel_scale, this->params.ang_vel_scale});
    
    this->params.torque_limits = torch::tensor({{20.0, 55.0, 55.0,    
                                                 20.0, 55.0, 55.0,
                                                 20.0, 55.0, 55.0,
                                                 20.0, 55.0, 55.0}});

    //                                              hip,    thigh,   calf
    this->params.default_dof_pos = torch::tensor({{-0.1000, 0.8000, -1.5000,   // front right
                                                    0.1000, 0.8000, -1.5000,   // front left
                                                   -0.1000, 1.0000, -1.5000,   // rear  right
                                                    0.1000, 1.0000, -1.5000}});// rear  left

    this->history_obs_buf = ObservationBuffer(1, this->params.num_observations, 6);

    output_torques = torch::tensor({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    output_dof_pos = params.default_dof_pos;
    plot_real_joint_pos.resize(12);
    plot_target_joint_pos.resize(12);

    loop_control = std::make_shared<LoopFunc>("loop_control", 0.002,    boost::bind(&RL_Real::RobotControl, this));
    loop_udpSend = std::make_shared<LoopFunc>("loop_udpSend", 0.002, 3, boost::bind(&RL_Real::UDPSend,      this));
    loop_udpRecv = std::make_shared<LoopFunc>("loop_udpRecv", 0.002, 3, boost::bind(&RL_Real::UDPRecv,      this));
    loop_rl      = std::make_shared<LoopFunc>("loop_rl"     , 0.02 ,    boost::bind(&RL_Real::RunModel,     this));

    loop_udpSend->start();
    loop_udpRecv->start();
    loop_control->start();
    
#ifdef PLOT
    loop_plot    = std::make_shared<LoopFunc>("loop_plot"   , 0.002,    boost::bind(&RL_Real::Plot,         this));
    loop_plot->start();
#endif
}

RL_Real::~RL_Real()
{
    loop_udpSend->shutdown();
    loop_udpRecv->shutdown();
    loop_control->shutdown();
    loop_rl->shutdown();
#ifdef PLOT
    loop_plot->shutdown();
#endif
    printf("exit\n");
}

void RL_Real::Plot()
{
    plot_t.push_back(motiontime);
    plt::cla();
    plt::clf();
    for(int i = 0; i < 12; ++i)
    {
        plot_real_joint_pos[i].push_back(state.motorState[i].q);
        plot_target_joint_pos[i].push_back(cmd.motorCmd[i].q);
        plt::subplot(4, 3, i+1);
        plt::named_plot("_real_joint_pos", plot_t, plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", plot_t, plot_target_joint_pos[i], "b");
        plt::xlim(motiontime-10000, motiontime);
    }
    // plt::legend();
    plt::pause(0.0001);
}

void RL_Real::RunModel()
{
    if(robot_state == STATE_RL_RUNNING)
    {
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
        // std::cout << "Execution time: " << duration << " microseconds" << std::endl;
        // start_time = std::chrono::high_resolution_clock::now();

        // printf("%f, %f, %f\n", 
        //     state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2]);
        // printf("%f, %f, %f, %f\n", 
        //     state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3], state.imu.quaternion[0]);
        // printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
        //     state.motorState[FL_0].q, state.motorState[FL_1].q, state.motorState[FL_2].q, 
        //     state.motorState[FR_0].q, state.motorState[FR_1].q, state.motorState[FR_2].q, 
        //     state.motorState[RL_0].q, state.motorState[RL_1].q, state.motorState[RL_2].q, 
        //     state.motorState[RR_0].q, state.motorState[RR_1].q, state.motorState[RR_2].q);
        // printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
        //     state.motorState[FL_0].dq, state.motorState[FL_1].dq, state.motorState[FL_2].dq, 
        //     state.motorState[FR_0].dq, state.motorState[FR_1].dq, state.motorState[FR_2].dq, 
        //     state.motorState[RL_0].dq, state.motorState[RL_1].dq, state.motorState[RL_2].dq, 
        //     state.motorState[RR_0].dq, state.motorState[RR_1].dq, state.motorState[RR_2].dq);
        
        this->obs.ang_vel = torch::tensor({{state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2]}});
        this->obs.commands = torch::tensor({{_keyData.ly, -_keyData.rx, -_keyData.lx}});
        this->obs.base_quat = torch::tensor({{state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3], state.imu.quaternion[0]}});
        this->obs.dof_pos = torch::tensor({{state.motorState[FL_0].q, state.motorState[FL_1].q, state.motorState[FL_2].q,
                                            state.motorState[FR_0].q, state.motorState[FR_1].q, state.motorState[FR_2].q,
                                            state.motorState[RL_0].q, state.motorState[RL_1].q, state.motorState[RL_2].q,
                                            state.motorState[RR_0].q, state.motorState[RR_1].q, state.motorState[RR_2].q}});
        this->obs.dof_vel = torch::tensor({{state.motorState[FL_0].dq, state.motorState[FL_1].dq, state.motorState[FL_2].dq,
                                            state.motorState[FR_0].dq, state.motorState[FR_1].dq, state.motorState[FR_2].dq,
                                            state.motorState[RL_0].dq, state.motorState[RL_1].dq, state.motorState[RL_2].dq,
                                            state.motorState[RR_0].dq, state.motorState[RR_1].dq, state.motorState[RR_2].dq}});
        
        torch::Tensor actions = this->Forward();

        output_torques = this->ComputeTorques(actions);
        output_dof_pos = this->ComputePosition(actions);
    }
    
}

torch::Tensor RL_Real::ComputeObservation()
{
    torch::Tensor obs = torch::cat({// (this->QuatRotateInverse(this->base_quat, this->lin_vel)) * this->params.lin_vel_scale,
                                    this->QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) * this->params.ang_vel_scale,
                                    this->QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec),
                                    this->obs.commands * this->params.commands_scale,
                                    (this->obs.dof_pos - this->params.default_dof_pos) * this->params.dof_pos_scale,
                                    this->obs.dof_vel * this->params.dof_vel_scale,
                                    this->obs.actions
                                    },1);
    obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);
    return obs;
}

torch::Tensor RL_Real::Forward()
{
    torch::Tensor obs = this->ComputeObservation();

    history_obs_buf.insert(obs);
    history_obs = history_obs_buf.get_obs_vec({0, 1, 2, 3, 4, 5});

    torch::Tensor encoding = this->encoder.forward({history_obs}).toTensor();

    torch::Tensor z = this->vq.forward({encoding}).toTensor();

    torch::Tensor actor_input = torch::cat({obs, z}, 1);

    torch::Tensor action = this->actor.forward({actor_input}).toTensor();

    this->obs.actions = action;
    torch::Tensor clamped = torch::clamp(action, -this->params.clip_actions, this->params.clip_actions);

    return clamped;
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
