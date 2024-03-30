#include "../include/rl_real_cyberdog.hpp"

#define PLOT

RL_Real rl_sar;

RL_Real::RL_Real() : CustomInterface(5000)
{
    start_time = std::chrono::high_resolution_clock::now();

    std::string actor_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/cyberdog/actor.pt";
    std::string encoder_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/cyberdog/encoder.pt";
    std::string vq_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/cyberdog/vq_layer.pt";

    this->actor = torch::jit::load(actor_path);
    this->encoder = torch::jit::load(encoder_path);
    this->vq = torch::jit::load(vq_path);
    this->InitObservations();
    
    this->params.num_observations = 45;
    this->params.clip_obs = 100.0;
    this->params.clip_actions = 100.0;
    this->params.damping = 1.0;
    this->params.stiffness = 30;
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
    this->params.default_dof_pos = torch::tensor({{ 0.1000, 0.8000, -1.5000,   // FL
                                                   -0.1000, 0.8000, -1.5000,   // FR
                                                    0.1000, 1.0000, -1.5000,   // RR
                                                   -0.1000, 1.0000, -1.5000}});// RL

    this->history_obs_buf = ObservationBuffer(1, this->params.num_observations, 6);

    output_torques = torch::tensor({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    output_dof_pos = params.default_dof_pos;
    plot_real_joint_pos.resize(12);
    plot_target_joint_pos.resize(12);

    loop_control = std::make_shared<LoopFunc>("loop_control", 0.002,    boost::bind(&RL_Real::RobotControl, this));
    loop_rl      = std::make_shared<LoopFunc>("loop_rl"     , 0.02 ,    boost::bind(&RL_Real::RunModel,     this));

    loop_control->start();
    
#ifdef PLOT
    loop_plot    = std::make_shared<LoopFunc>("loop_plot"   , 0.002,    boost::bind(&RL_Real::Plot,         this));
    loop_plot->start();
#endif
    _keyboardThread = std::thread(&RL_Real::run_keyboard, this);
}

RL_Real::~RL_Real()
{
    loop_control->shutdown();
    loop_rl->shutdown();
#ifdef PLOT
    loop_plot->shutdown();
#endif

    system("ssh root@192.168.55.233 \"ps | grep -E 'manager|ctrl|imu_online' | grep -v grep | awk '{print \\$1}' | xargs kill -9\"");
    system("ssh root@192.168.55.233 \"export LD_LIBRARY_PATH=/mnt/UDISK/robot-software/build;/mnt/UDISK/manager /mnt/UDISK/ >> /mnt/UDISK/manager_log/manager.log 2>&1 &\"");

    printf("exit\n");
}

void RL_Real::RobotControl()
{
    std::cout << "robot_state" << keyboard.robot_state
              << " x" << keyboard.x << " y" << keyboard.y << " yaw" << keyboard.yaw
              << "\r";
    motiontime++;

    // waiting
    if(robot_state == STATE_WAITING)
    {
        for(int i = 0; i < 12; ++i)
        {
            cyberdogCmd.q_des[i] = cyberdogData.q[i];
        }
        if(keyboard.robot_state == STATE_POS_GETUP)
        {
            keyboard.robot_state = STATE_WAITING;
            getup_percent = 0.0;
            for(int i = 0; i < 12; ++i)
            {
                now_pos[i] = cyberdogData.q[i];
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
                cyberdogCmd.q_des[i] = (1 - getup_percent) * now_pos[i] + getup_percent * params.default_dof_pos[0][dof_mapping[i]].item<double>();
                cyberdogCmd.qd_des[i] = 0;
                cyberdogCmd.kp_des[i] = 200;
                cyberdogCmd.kd_des[i] = 10;
                cyberdogCmd.tau_des[i] = 0;
            }
            printf("getting up %.3f%%\r", getup_percent*100.0);
        }
        if(keyboard.robot_state == STATE_RL_INIT)
        {
            keyboard.robot_state = STATE_WAITING;
            robot_state = STATE_RL_INIT;
        }
        else if(keyboard.robot_state == STATE_POS_GETDOWN)
        {
            keyboard.robot_state = STATE_WAITING;
            getdown_percent = 0.0;
            for(int i = 0; i < 12; ++i)
            {
                now_pos[i] = cyberdogData.q[i];
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
            // cyberdogCmd.q_des[i] = 0;
            cyberdogCmd.q_des[i] = output_dof_pos[0][dof_mapping[i]].item<double>();
            cyberdogCmd.qd_des[i] = 0;
            cyberdogCmd.kp_des[i] = params.stiffness;
            cyberdogCmd.kd_des[i] = params.damping;
            // cyberdogCmd.tau_des[i] = output_torques[0][dof_mapping[i]].item<double>();
            cyberdogCmd.tau_des[i] = 0;
        }
        if(keyboard.robot_state == STATE_POS_GETDOWN)
        {
            keyboard.robot_state = STATE_WAITING;
            getdown_percent = 0.0;
            for(int i = 0; i < 12; ++i)
            {
                now_pos[i] = cyberdogData.q[i];
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
                cyberdogCmd.q_des[i] = (1 - getdown_percent) * now_pos[i] + getdown_percent * start_pos[i];
                cyberdogCmd.qd_des[i] = 0;
                cyberdogCmd.kp_des[i] = 200;
                cyberdogCmd.kd_des[i] = 10;
                cyberdogCmd.tau_des[i] = 0;
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
}

void RL_Real::UserCode()
{
	cyberdogData = robot_data;
	motor_cmd = cyberdogCmd;
}

static bool kbhit()
{
    termios term;
    tcgetattr(0, &term);
    
    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);
    
    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);
    
    tcsetattr(0, TCSANOW, &term);
    
    return byteswaiting > 0;
}
void RL_Real::run_keyboard()
{
    int c;
    // Check for keyboard input
    while(true)
    {
        if(kbhit())
        {
            c = fgetc(stdin);
            switch(c)
            {
                case '0':
                    keyboard.robot_state = STATE_POS_GETUP;
                    break;
                case 'p':
                    keyboard.robot_state = STATE_RL_INIT;
                    break;
                case '1':
                    keyboard.robot_state = STATE_POS_GETDOWN;
                    break;
                case 'q':
                    break;
                case 'w':
                    keyboard.x += 0.1;
                    break;
                case 's':
                    keyboard.x -= 0.1;
                    break;
                case 'a':
                    keyboard.yaw += 0.1;
                    break;
                case 'd':
                    keyboard.yaw -= 0.1;
                    break;
                case 'i':
                    break;
                case 'k':
                    break;
                case 'j':
                    keyboard.y += 0.1;
                    break;
                case 'l':
                    keyboard.y -= 0.1;
                    break;
                case ' ':
                    keyboard.x = 0;
                    keyboard.y = 0;
                    keyboard.yaw = 0;
                    break;
                default:
                    break;
            }
        }
        usleep(10000);
    } 
}

void RL_Real::RunModel()
{
    if(robot_state == STATE_RL_RUNNING)
    {
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
        // std::cout << "Execution time: " << duration << " microseconds" << std::endl;
        // start_time = std::chrono::high_resolution_clock::now();

        // printf("%f, %f, %f\n", 
        //     cyberdogData.omega[0], cyberdogData.omega[1], cyberdogData.omega[2]);
        // printf("%f, %f, %f, %f\n", 
        //     cyberdogData.quat[1], cyberdogData.quat[2], cyberdogData.quat[3], cyberdogData.quat[0]);
        // printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
        //     cyberdogData.q[3], cyberdogData.q[4], cyberdogData.q[5],
        //     cyberdogData.q[0], cyberdogData.q[1], cyberdogData.q[2],
        //     cyberdogData.q[9], cyberdogData.q[10], cyberdogData.q[11],
        //     cyberdogData.q[6], cyberdogData.q[7], cyberdogData.q[8]);
        // printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
        //     cyberdogData.qd[3], cyberdogData.qd[4], cyberdogData.qd[5],
        //     cyberdogData.qd[0], cyberdogData.qd[1], cyberdogData.qd[2],
        //     cyberdogData.qd[9], cyberdogData.qd[10], cyberdogData.qd[11],
        //     cyberdogData.qd[6], cyberdogData.qd[7], cyberdogData.qd[8]);
        
        this->obs.ang_vel = torch::tensor({{cyberdogData.omega[0], cyberdogData.omega[1], cyberdogData.omega[2]}});
        this->obs.commands = torch::tensor({{keyboard.x, keyboard.y, keyboard.yaw}});
        this->obs.base_quat = torch::tensor({{cyberdogData.quat[1], cyberdogData.quat[2], cyberdogData.quat[3], cyberdogData.quat[0]}});
        this->obs.dof_pos = torch::tensor({{cyberdogData.q[3], cyberdogData.q[4], cyberdogData.q[5],
                                            cyberdogData.q[0], cyberdogData.q[1], cyberdogData.q[2],
                                            cyberdogData.q[9], cyberdogData.q[10], cyberdogData.q[11],
                                            cyberdogData.q[6], cyberdogData.q[7], cyberdogData.q[8]}});
        this->obs.dof_vel = torch::tensor({{cyberdogData.qd[3], cyberdogData.qd[4], cyberdogData.qd[5],
                                            cyberdogData.qd[0], cyberdogData.qd[1], cyberdogData.qd[2],
                                            cyberdogData.qd[9], cyberdogData.qd[10], cyberdogData.qd[11],
                                            cyberdogData.qd[6], cyberdogData.qd[7], cyberdogData.qd[8]}});
        
        torch::Tensor actions = this->Forward();

        for (int i : hip_scale_reduction_indices)
        {
            actions[0][i] *= this->params.hip_scale_reduction;
        }

        output_torques = this->ComputeTorques(actions);
        output_dof_pos = this->ComputePosition(actions);
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

void RL_Real::Plot()
{
    plot_t.push_back(motiontime);
    plt::cla();
    plt::clf();
    for(int i = 0; i < 12; ++i)
    {
        plot_real_joint_pos[i].push_back(cyberdogData.q[i]);
        plot_target_joint_pos[i].push_back(cyberdogCmd.q_des[i]);
        plt::subplot(4, 3, i+1);
        plt::named_plot("_real_joint_pos", plot_t, plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", plot_t, plot_target_joint_pos[i], "b");
        plt::xlim(motiontime-10000, motiontime);
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
