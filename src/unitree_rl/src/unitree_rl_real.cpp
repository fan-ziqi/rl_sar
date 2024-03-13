#include "../include/unitree_rl_real.hpp"
#include <ros/package.h>

void Unitree_RL::UDPRecv()
{ 
    udp.Recv();
}

void Unitree_RL::UDPSend()
{  
    udp.Send();
}

void Unitree_RL::RobotControl()
{
    motiontime++;
    udp.GetRecv(state);

    if( motiontime < 50)
    {
        for(int i = 0; i < 12; ++i)
        {
            cmd.motorCmd[i].q = state.motorState[i].q;
            _startPos[i] = state.motorState[i].q;
        }
    }

    if( motiontime >= 50 && _percent != 1)
    {
        _percent += (float) 1 / 100;
        _percent = _percent > 1 ? 1 : _percent;
        for(int i = 0; i < 12; ++i)
        {
            cmd.motorCmd[i].q = (1 - _percent) * _startPos[i] + _percent * _targetPos[i];
            cmd.motorCmd[i].dq = 0;
            cmd.motorCmd[i].Kp = 50;
            cmd.motorCmd[i].Kd = 3;
            cmd.motorCmd[i].tau = 0;
        }
    }
    if(_percent == 1 && !init_done)
    {
        init_done = true;
        this->init_observations();
        loop_rl->start();
        std::cout << "init done" << std::endl;
        motiontime = 0;
    }

    if(init_done)
    {
        if( motiontime < 50)
        {
            for(int i = 0; i < 12; ++i)
            {
                cmd.motorCmd[i].q = _targetPos[i];
                cmd.motorCmd[i].dq = 0;
                cmd.motorCmd[i].Kp = 50;
                cmd.motorCmd[i].Kd = 3;
                cmd.motorCmd[i].tau = 0;
                _startPos[i] = state.motorState[i].q;
            }
        }
        if( motiontime >= 50)
        {
            for (int i = 0; i < 12; ++i)
            {
                float torque = torques[0][i].item<double>();
                // if(torque > 5.0f) torque = 5.0f;
                // if(torque < -5.0f) torque = -5.0f;

                cmd.motorCmd[i].q = PosStopF;
                cmd.motorCmd[i].dq = VelStopF;
                cmd.motorCmd[i].Kp = 0;
                cmd.motorCmd[i].Kd = 0;
                cmd.motorCmd[i].tau = torque;
            }
        }
    }

    // safe.PowerProtect(cmd, state, 1);
    udp.SetSend(cmd);
}

Unitree_RL::Unitree_RL() : safe(LeggedType::A1), udp(LOWLEVEL)
{
    udp.InitCmdData(cmd);

    start_time = std::chrono::high_resolution_clock::now();

    cmd_vel = geometry_msgs::Twist();

    torque_commands.resize(12);

    std::string package_name = "unitree_rl";
    std::string actor_path = ros::package::getPath(package_name) + "/models/actor.pt";
    std::string encoder_path = ros::package::getPath(package_name) + "/models/encoder.pt";
    std::string vq_path = ros::package::getPath(package_name) + "/models/vq_layer.pt";

    this->actor = torch::jit::load(actor_path);
    this->encoder = torch::jit::load(encoder_path);
    this->vq = torch::jit::load(vq_path);
    this->init_observations();
    
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

    //                                             hip,    thigh,   calf
    this->params.default_dof_pos = torch::tensor({{-0.1000, 0.8000, -1.5000,   // front right
                                                    0.1000, 0.8000, -1.5000,   // front left
                                                   -0.1000, 1.0000, -1.5000,   // rear  right
                                                    0.1000, 1.0000, -1.5000}});// rear  left

    this->history_obs_buf = ObservationBuffer(1, this->params.num_observations, 6);

    // InitEnvironment();
    loop_control = std::make_shared<LoopFunc>("control_loop", 0.02 ,    boost::bind(&Unitree_RL::RobotControl, this));
    loop_udpSend = std::make_shared<LoopFunc>("udp_send"    , 0.002, 3, boost::bind(&Unitree_RL::UDPSend,      this));
    loop_udpRecv = std::make_shared<LoopFunc>("udp_recv"    , 0.002, 3, boost::bind(&Unitree_RL::UDPRecv,      this));
    loop_rl      = std::make_shared<LoopFunc>("rl_loop"     , 0.02 ,    boost::bind(&Unitree_RL::runModel,     this));

    loop_udpSend->start();
    loop_udpRecv->start();
    loop_control->start();
}

void Unitree_RL::cmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel = *msg;
}

// void Unitree_RL::runModel(const ros::TimerEvent &event)
void Unitree_RL::runModel()
{
    if(init_done)
    {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
        // std::cout << "Execution time: " << duration << " microseconds" << std::endl;
        start_time = std::chrono::high_resolution_clock::now();

        // printf("%f, %f, %f\n", state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2]);
        // printf("%f, %f, %f, %f\n", state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3], state.imu.quaternion[0]);
        // printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", state.motorState[FL_0].q, state.motorState[FL_1].q, state.motorState[FL_2].q, state.motorState[FR_0].q, state.motorState[FR_1].q, state.motorState[FR_2].q, state.motorState[RL_0].q, state.motorState[RL_1].q, state.motorState[RL_2].q, state.motorState[RR_0].q, state.motorState[RR_1].q, state.motorState[RR_2].q);
        // printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", state.motorState[FL_0].dq, state.motorState[FL_1].dq, state.motorState[FL_2].dq, state.motorState[FR_0].dq, state.motorState[FR_1].dq, state.motorState[FR_2].dq, state.motorState[RL_0].dq, state.motorState[RL_1].dq, state.motorState[RL_2].dq, state.motorState[RR_0].dq, state.motorState[RR_1].dq, state.motorState[RR_2].dq);

        this->obs.ang_vel = torch::tensor({{state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2]}});
        this->obs.commands = torch::tensor({{cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z}});
        this->obs.base_quat = torch::tensor({{state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3], state.imu.quaternion[0]}});
        this->obs.dof_pos = torch::tensor({{state.motorState[FR_0].q, state.motorState[FR_1].q, state.motorState[FR_2].q,
                                            state.motorState[FL_0].q, state.motorState[FL_1].q, state.motorState[FL_2].q,
                                            state.motorState[RR_0].q, state.motorState[RR_1].q, state.motorState[RR_2].q,
                                            state.motorState[RL_0].q, state.motorState[RL_1].q, state.motorState[RL_2].q}});
        this->obs.dof_vel = torch::tensor({{state.motorState[FR_0].dq, state.motorState[FR_1].dq, state.motorState[FR_2].dq,
                                            state.motorState[FL_0].dq, state.motorState[FL_1].dq, state.motorState[FL_2].dq,
                                            state.motorState[RR_0].dq, state.motorState[RR_1].dq, state.motorState[RR_2].dq,
                                            state.motorState[RL_0].dq, state.motorState[RL_1].dq, state.motorState[RL_2].dq}});
        
        torch::Tensor actions = this->forward();
        torques = this->compute_torques(actions);
    }
    
}

torch::Tensor Unitree_RL::compute_observation()
{
    torch::Tensor ang_vel = this->quat_rotate_inverse(this->obs.base_quat, this->obs.ang_vel);
    // float ang_vel_temp = ang_vel[0][0].item<double>();
    // ang_vel[0][0] = ang_vel[0][1];
    // ang_vel[0][1] = ang_vel_temp;

    torch::Tensor grav = this->quat_rotate_inverse(this->obs.base_quat, this->obs.gravity_vec);
    // float grav_temp = grav[0][0].item<double>();
    // grav[0][0] = grav[0][1];
    // grav[0][1] = grav_temp;

    torch::Tensor obs = torch::cat({// (this->quat_rotate_inverse(this->base_quat, this->lin_vel)) * this->params.lin_vel_scale,
                                    ang_vel * this->params.ang_vel_scale,
                                    grav,
                                    this->obs.commands * this->params.commands_scale,
                                    (this->obs.dof_pos - this->params.default_dof_pos) * this->params.dof_pos_scale,
                                    this->obs.dof_vel * this->params.dof_vel_scale,
                                    this->obs.actions},
                                   1);
    obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);
    return obs;
}

torch::Tensor Unitree_RL::forward()
{
    torch::Tensor obs = this->compute_observation();

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



int main(int argc, char **argv)
{
    Unitree_RL unitree_rl;

    while(1){
        sleep(10);
    };

    return 0;
}
