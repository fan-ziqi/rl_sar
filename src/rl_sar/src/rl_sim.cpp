#include "../include/rl_sim.hpp"

#define ROBOT_NAME "cyberdog1"

// #define PLOT

RL_Sim::RL_Sim()
{
    ReadYaml(ROBOT_NAME);

    ros::NodeHandle nh;
    start_time = std::chrono::high_resolution_clock::now();

    cmd_vel = geometry_msgs::Twist();

    motor_commands.resize(12);

    std::string actor_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + ROBOT_NAME + "/actor.pt";
    std::string encoder_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + ROBOT_NAME + "/encoder.pt";
    std::string vq_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + ROBOT_NAME + "/vq_layer.pt";

    this->actor = torch::jit::load(actor_path);
    this->encoder = torch::jit::load(encoder_path);
    this->vq = torch::jit::load(vq_path);
    this->InitObservations();
    this->InitOutputs();

    this->history_obs_buf = ObservationBuffer(1, this->params.num_observations, 6);

    joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    joint_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    plot_real_joint_pos.resize(12);
    plot_target_joint_pos.resize(12);

    cmd_vel_subscriber_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Sim::CmdvelCallback, this);

    nh.param<std::string>("ros_namespace", ros_namespace, "");

    joint_names = {
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    };

    for (int i = 0; i < 12; ++i)
    {
        torque_publishers[joint_names[i]] = nh.advertise<unitree_legged_msgs::MotorCmd>(
            ros_namespace + joint_names[i].substr(0, joint_names[i].size() - 6) + "_controller/command", 10);
    }

    model_state_subscriber_ = nh.subscribe<gazebo_msgs::ModelStates>(
        "/gazebo/model_states", 10, &RL_Sim::ModelStatesCallback, this);

    joint_state_subscriber_ = nh.subscribe<sensor_msgs::JointState>(
        ros_namespace + "joint_states", 10, &RL_Sim::JointStatesCallback, this);

    loop_control = std::make_shared<LoopFunc>("loop_control", 0.002,    boost::bind(&RL_Sim::RobotControl, this));
    loop_rl      = std::make_shared<LoopFunc>("loop_rl"     , 0.02 ,    boost::bind(&RL_Sim::RunModel,     this));

    loop_control->start();
    loop_rl->start();
#ifdef PLOT
    loop_plot    = std::make_shared<LoopFunc>("loop_plot"   , 0.002,    boost::bind(&RL_Sim::Plot,         this));
    loop_plot->start();
#endif
}

RL_Sim::~RL_Sim()
{
    loop_control->shutdown();
    loop_rl->shutdown();
#ifdef PLOT
    loop_plot->shutdown();
#endif
    printf("exit\n");
}

void RL_Sim::RobotControl()
{
    motiontime++;
    for (int i = 0; i < 12; ++i)
    {
        motor_commands[i].mode = 0x0A;
        motor_commands[i].q = output_dof_pos[0][i].item<double>();
        motor_commands[i].dq = 0;
        motor_commands[i].Kp = params.stiffness;
        motor_commands[i].Kd = params.damping;
        // motor_commands[i].tau = output_torques[0][i].item<double>();
        motor_commands[i].tau = 0;

        torque_publishers[joint_names[i]].publish(motor_commands[i]);
    }
}

void RL_Sim::ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{

    vel = msg->twist[2];
    pose = msg->pose[2];
}

void RL_Sim::CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel = *msg;
}

void RL_Sim::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_positions = msg->position;
    joint_velocities = msg->velocity;
}

void RL_Sim::RunModel()
{
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    // std::cout << "Execution time: " << duration << " microseconds" << std::endl;
    // start_time = std::chrono::high_resolution_clock::now();

    // printf("%f, %f, %f\n", 
    //     vel.angular.x, vel.angular.y, vel.angular.z);
    // printf("%f, %f, %f, %f\n", 
    //     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    // printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
    //     joint_positions[1], joint_positions[2], joint_positions[0],
    //     joint_positions[4], joint_positions[5], joint_positions[3],
    //     joint_positions[7], joint_positions[8], joint_positions[6],
    //     joint_positions[10], joint_positions[11], joint_positions[9]);
    // printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
    //     joint_velocities[1], joint_velocities[2], joint_velocities[0],
    //     joint_velocities[4], joint_velocities[5], joint_velocities[3],
    //     joint_velocities[7], joint_velocities[8], joint_velocities[6],
    //     joint_velocities[10], joint_velocities[11], joint_velocities[9]);

    // this->obs.lin_vel = torch::tensor({{vel.linear.x, vel.linear.y, vel.linear.z}});
    this->obs.ang_vel = torch::tensor({{vel.angular.x, vel.angular.y, vel.angular.z}});
    this->obs.commands = torch::tensor({{cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z}});
    this->obs.base_quat = torch::tensor({{pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}});
    this->obs.dof_pos = torch::tensor({{joint_positions[1], joint_positions[2], joint_positions[0],
                                        joint_positions[4], joint_positions[5], joint_positions[3],
                                        joint_positions[7], joint_positions[8], joint_positions[6],
                                        joint_positions[10], joint_positions[11], joint_positions[9]}});
    this->obs.dof_vel = torch::tensor({{joint_velocities[1], joint_velocities[2], joint_velocities[0],
                                        joint_velocities[4], joint_velocities[5], joint_velocities[3],
                                        joint_velocities[7], joint_velocities[8], joint_velocities[6],
                                        joint_velocities[10], joint_velocities[11], joint_velocities[9]}});

    torch::Tensor actions = this->Forward();

    for (int i : hip_scale_reduction_indices)
    {
        actions[0][i] *= this->params.hip_scale_reduction;
    }

    output_torques = this->ComputeTorques(actions);
    output_dof_pos = this->ComputePosition(actions);
}

torch::Tensor RL_Sim::ComputeObservation()
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

torch::Tensor RL_Sim::Forward()
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

void RL_Sim::Plot()
{
    int dof_mapping[13] = {1, 2, 0, 4, 5, 3, 7, 8, 6, 10, 11, 9};
    plot_t.push_back(motiontime);
    plt::cla();
    plt::clf();
    for(int i = 0; i < 12; ++i)
    {
        plot_real_joint_pos[i].push_back(joint_positions[dof_mapping[i]]);
        plot_target_joint_pos[i].push_back(motor_commands[i].q);
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
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);

    ros::init(argc, argv, "rl_sar");

    RL_Sim rl_sar;

    ros::spin();

    return 0;
}
