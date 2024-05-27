#include "../include/rl_sim.hpp"

// #define PLOT
// #define CSV_LOGGER

RL_Sim::RL_Sim()
{
    ros::NodeHandle nh;

    // read params from yaml
    nh.param<std::string>("robot_name", robot_name, "");
    ReadYaml(robot_name);

    // history
    nh.param<bool>("use_history", use_history, "");
    if(use_history)
    {
        this->history_obs_buf = ObservationBuffer(1, this->params.num_observations, 6);
    }

    // Due to the fact that the robot_state_publisher sorts the joint names alphabetically,
    // the mapping table is established according to the order defined in the YAML file
    std::vector<std::string> sorted_joint_controller_names = params.joint_controller_names;
    std::sort(sorted_joint_controller_names.begin(), sorted_joint_controller_names.end());
    for(size_t i = 0; i < params.joint_controller_names.size(); ++i)
    {
        sorted_to_original_index[sorted_joint_controller_names[i]] = i;
    }
    mapped_joint_positions = std::vector<double>(params.num_of_dofs, 0.0);
    mapped_joint_velocities = std::vector<double>(params.num_of_dofs, 0.0);
    mapped_joint_efforts = std::vector<double>(params.num_of_dofs, 0.0);

    // init
    torch::autograd::GradMode::set_enabled(false);
    motor_commands.resize(params.num_of_dofs);
    start_pos.resize(params.num_of_dofs);
    now_pos.resize(params.num_of_dofs);
    this->InitObservations();
    this->InitOutputs();
    this->InitKeyboard();

    // model
    std::string model_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + robot_name + "/" + this->params.model_name;
    this->model = torch::jit::load(model_path);

    // publisher
    nh.param<std::string>("ros_namespace", ros_namespace, "");
    for (int i = 0; i < params.num_of_dofs; ++i)
    {
        // joint need to rename as xxx_joint
        torque_publishers[params.joint_controller_names[i]] = nh.advertise<robot_msgs::MotorCommand>(
            ros_namespace + params.joint_controller_names[i] + "/command", 10);
    }

    // subscriber
    cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Sim::CmdvelCallback, this);
    model_state_subscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &RL_Sim::ModelStatesCallback, this);
    joint_state_subscriber = nh.subscribe<sensor_msgs::JointState>(ros_namespace + "joint_states", 10, &RL_Sim::JointStatesCallback, this);

    // service
    gazebo_reset_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    
    // loop
    loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05 ,    boost::bind(&RL_Sim::RunKeyboard,  this));
    loop_control  = std::make_shared<LoopFunc>("loop_control" , 0.002,    boost::bind(&RL_Sim::RobotControl, this));
    loop_rl       = std::make_shared<LoopFunc>("loop_rl"      , 0.02 ,    boost::bind(&RL_Sim::RunModel,     this));
    loop_keyboard->start();
    loop_control->start();
    loop_rl->start();

#ifdef PLOT
    plot_t = std::vector<int>(plot_size, 0);
    plot_real_joint_pos.resize(params.num_of_dofs);
    plot_target_joint_pos.resize(params.num_of_dofs);
    for(auto& vector : plot_real_joint_pos) { vector = std::vector<double>(plot_size, 0); }
    for(auto& vector : plot_target_joint_pos) { vector = std::vector<double>(plot_size, 0); }
    loop_plot    = std::make_shared<LoopFunc>("loop_plot"   , 0.002,    boost::bind(&RL_Sim::Plot,         this));
    loop_plot->start();
#endif
#ifdef CSV_LOGGER
    CSVInit(robot_name);
#endif
}

RL_Sim::~RL_Sim()
{
    loop_keyboard->shutdown();
    loop_control->shutdown();
    loop_rl->shutdown();
#ifdef PLOT
    loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::GetState(RobotState<double> *state)
{
    state->imu.quaternion[3] = pose.orientation.w;
    state->imu.quaternion[0] = pose.orientation.x;
    state->imu.quaternion[1] = pose.orientation.y;
    state->imu.quaternion[2] = pose.orientation.z;

    state->imu.gyroscope[0] = vel.angular.x;
    state->imu.gyroscope[1] = vel.angular.y;
    state->imu.gyroscope[2] = vel.angular.z;

    // state->imu.accelerometer

    for(int i = 0; i < params.num_of_dofs; ++i)
    {
        state->motor_state.q[i] = mapped_joint_positions[i];
        state->motor_state.dq[i] = mapped_joint_velocities[i];
        state->motor_state.tauEst[i] = mapped_joint_efforts[i];
    }
}

void RL_Sim::SetCommand(const RobotCommand<double> *command)
{
    for(int i = 0; i < params.num_of_dofs; ++i)
    {
        motor_commands[i].q = command->motor_command.q[i];
        motor_commands[i].dq = command->motor_command.dq[i];
        motor_commands[i].kp = command->motor_command.kp[i];
        motor_commands[i].kd = command->motor_command.kd[i];
        motor_commands[i].tau = command->motor_command.tau[i];
    }
    
    for(int i = 0; i < params.num_of_dofs; ++i)
    {
        torque_publishers[params.joint_controller_names[i]].publish(motor_commands[i]);
    }
}

void RL_Sim::RobotControl()
{
    motiontime++;

    if(keyboard.keyboard_state == STATE_RESET_SIMULATION)
    {
        keyboard.keyboard_state = STATE_WAITING;
        std_srvs::Empty srv;
        gazebo_reset_client.call(srv);
    }

    GetState(&robot_state);
    StateController(&robot_state, &robot_command);
    SetCommand(&robot_command);
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
        target_data[i] = source_data[sorted_to_original_index[params.joint_controller_names[i]]];
    }
}

void RL_Sim::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    MapData(msg->position, mapped_joint_positions);
    MapData(msg->velocity, mapped_joint_velocities);
    MapData(msg->effort, mapped_joint_efforts);
}

void RL_Sim::RunModel()
{
    if(running_state == STATE_RL_RUNNING)
    {
        // this->obs.lin_vel = torch::tensor({{vel.linear.x, vel.linear.y, vel.linear.z}});
        this->obs.ang_vel = torch::tensor(robot_state.imu.gyroscope).unsqueeze(0);
        // this->obs.commands = torch::tensor({{cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z}});
        this->obs.commands = torch::tensor({{keyboard.x, keyboard.y, keyboard.yaw}});
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
        torch::Tensor tau_est = torch::tensor(mapped_joint_efforts).unsqueeze(0);
        CSVLogger(output_torques, tau_est, this->obs.dof_pos, output_dof_pos, this->obs.dof_vel);
#endif
    }
}

torch::Tensor RL_Sim::ComputeObservation()
{
    torch::Tensor obs = torch::cat({// this->obs.lin_vel * this->params.lin_vel_scale,
                                    this->QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) * this->params.ang_vel_scale,
                                    // this->obs.ang_vel * this->params.ang_vel_scale, // TODO
                                    this->QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec),
                                    this->obs.commands * this->params.commands_scale,
                                    (this->obs.dof_pos - this->params.default_dof_pos) * this->params.dof_pos_scale,
                                    this->obs.dof_vel * this->params.dof_vel_scale,
                                    this->obs.actions
                                    },1);
    torch::Tensor clamped_obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);
    return clamped_obs;
}

torch::Tensor RL_Sim::Forward()
{
    torch::autograd::GradMode::set_enabled(false);

    torch::Tensor clamped_obs = this->ComputeObservation();

    torch::Tensor actions;

    if(use_history)
    {
        history_obs_buf.insert(clamped_obs);
        history_obs = history_obs_buf.get_obs_vec({0, 1, 2, 3, 4, 5});
        actions = this->model.forward({history_obs}).toTensor();
    }
    else
    {
        actions = this->model.forward({clamped_obs}).toTensor();
    }  

    torch::Tensor clamped_actions = torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);

    return clamped_actions;
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
        plot_real_joint_pos[i].push_back(mapped_joint_positions[i]);
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
