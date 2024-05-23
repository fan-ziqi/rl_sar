#include "../include/rl_sim.hpp"

#define ROBOT_NAME "a1"

// #define PLOT
// #define CSV_LOGGER
#define USE_HISTORY

RL_Sim::RL_Sim()
{
    torch::autograd::GradMode::set_enabled(false);

    ReadYaml(ROBOT_NAME);

    // Due to the fact that the robot_state_publisher sorts the joint names alphabetically,
    // the mapping table is established according to the order defined in the YAML file
    std::vector<std::string> sorted_joint_names = params.joint_names;
    std::sort(sorted_joint_names.begin(), sorted_joint_names.end());
    for(size_t i = 0; i < params.joint_names.size(); ++i)
    {
        sorted_to_original_index[sorted_joint_names[i]] = i;
    }

    ros::NodeHandle nh;
    start_time = std::chrono::high_resolution_clock::now();

    cmd_vel = geometry_msgs::Twist();

    motor_commands.resize(params.num_of_dofs);

    std::string model_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + ROBOT_NAME + "/" + this->params.model_name;
    this->model = torch::jit::load(model_path);

    this->InitObservations();
    this->InitOutputs();

#ifdef USE_HISTORY
    this->history_obs_buf = ObservationBuffer(1, this->params.num_observations, 6);
#endif

    joint_positions = std::vector<double>(params.num_of_dofs, 0.0);
    joint_velocities = std::vector<double>(params.num_of_dofs, 0.0);
    joint_efforts = std::vector<double>(params.num_of_dofs, 0.0);
    
    plot_t = std::vector<int>(plot_size, 0);
    plot_real_joint_pos.resize(params.num_of_dofs);
    plot_target_joint_pos.resize(params.num_of_dofs);
    for(auto& vector : plot_real_joint_pos) { vector = std::vector<double>(plot_size, 0); }
    for(auto& vector : plot_target_joint_pos) { vector = std::vector<double>(plot_size, 0); }

    cmd_vel_subscriber_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Sim::CmdvelCallback, this);

    nh.param<std::string>("ros_namespace", ros_namespace, "");

    for (int i = 0; i < params.num_of_dofs; ++i)
    {
        torque_publishers[params.joint_names[i]] = nh.advertise<unitree_legged_msgs::MotorCmd>(
            ros_namespace + params.joint_names[i].substr(0, params.joint_names[i].size() - 6) + "_controller/command", 10);
    }

    model_state_subscriber_ = nh.subscribe<gazebo_msgs::ModelStates>(
        "/gazebo/model_states", 10, &RL_Sim::ModelStatesCallback, this);

    joint_state_subscriber_ = nh.subscribe<sensor_msgs::JointState>(
        ros_namespace + "joint_states", 10, &RL_Sim::JointStatesCallback, this);

    loop_control = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("loop_control", 0.002,    boost::bind(&RL_Sim::RobotControl, this));
    loop_rl      = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("loop_rl"     , 0.02 ,    boost::bind(&RL_Sim::RunModel,     this));

    loop_control->start();
    loop_rl->start();
#ifdef PLOT
    loop_plot    = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("loop_plot"   , 0.002,    boost::bind(&RL_Sim::Plot,         this));
    loop_plot->start();
#endif

#ifdef CSV_LOGGER
    CSVInit(ROBOT_NAME);
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
    for (int i = 0; i < params.num_of_dofs; ++i)
    {
        motor_commands[i].mode = 0x0A;
        motor_commands[i].q = output_dof_pos[0][i].item<double>();
        motor_commands[i].dq = 0;
        // motor_commands[i].Kp = params.stiffness;
        // motor_commands[i].Kd = params.damping;
        motor_commands[i].Kp = params.p_gains[0][i].item<double>();
        motor_commands[i].Kd = params.d_gains[0][i].item<double>();
        // motor_commands[i].tau = output_torques[0][i].item<double>();
        motor_commands[i].tau = 0;

        torque_publishers[params.joint_names[i]].publish(motor_commands[i]);
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

void RL_Sim::MapData(const std::vector<double>& source_data, std::vector<double>& target_data)
{
    for(size_t i = 0; i < source_data.size(); ++i)
    {
        target_data[i] = source_data[sorted_to_original_index[params.joint_names[i]]];
    }
}

void RL_Sim::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    MapData(msg->position, joint_positions);
    MapData(msg->velocity, joint_velocities);
    MapData(msg->effort, joint_efforts);
}

void RL_Sim::RunModel()
{
    // this->obs.lin_vel = torch::tensor({{vel.linear.x, vel.linear.y, vel.linear.z}});
    this->obs.ang_vel = torch::tensor({{vel.angular.x, vel.angular.y, vel.angular.z}});
    this->obs.commands = torch::tensor({{cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z}});
    this->obs.base_quat = torch::tensor({{pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}});
    this->obs.dof_pos = torch::tensor(joint_positions).unsqueeze(0);
    this->obs.dof_vel = torch::tensor(joint_velocities).unsqueeze(0);

    torch::Tensor actions = this->Forward();

    for (int i : this->params.hip_scale_reduction_indices)
    {
        actions[0][i] *= this->params.hip_scale_reduction;
    }

    output_torques = this->ComputeTorques(actions);
    output_dof_pos = this->ComputePosition(actions);

#ifdef CSV_LOGGER
    torch::Tensor tau_est = torch::tensor(joint_efforts).unsqueeze(0);
    CSVLogger(output_torques, tau_est, this->obs.dof_pos, output_dof_pos, this->obs.dof_vel);
#endif
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

#ifdef USE_HISTORY
    history_obs_buf.insert(obs);
    history_obs = history_obs_buf.get_obs_vec({0, 1, 2, 3, 4, 5});
    torch::Tensor action = this->model.forward({history_obs}).toTensor();
#else
    torch::Tensor action = this->model.forward({obs}).toTensor();
#endif

    this->obs.actions = action;
    torch::Tensor clamped = torch::clamp(action, -this->params.clip_actions, this->params.clip_actions);

    return clamped;
}

void RL_Sim::Plot()
{
    plot_t.erase(plot_t.begin());
    plot_t.push_back(motiontime);
    plt::cla();
    plt::clf();
    for(int i = 0; i < params.num_of_dofs; ++i)
    {
        plot_real_joint_pos[i].erase(plot_real_joint_pos[i].begin());
        plot_target_joint_pos[i].erase(plot_target_joint_pos[i].begin());
        plot_real_joint_pos[i].push_back(joint_positions[i]);
        plot_target_joint_pos[i].push_back(motor_commands[i].q);
        plt::subplot(4, 3, i+1);
        plt::named_plot("_real_joint_pos", plot_t, plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", plot_t, plot_target_joint_pos[i], "b");
        plt::xlim(plot_t.front(), plot_t.back());
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
