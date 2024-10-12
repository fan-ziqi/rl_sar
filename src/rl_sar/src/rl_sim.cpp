#include "rl_sim.hpp"

// #define PLOT
// #define CSV_LOGGER

RL_Sim::RL_Sim()
{
    ros::NodeHandle nh;

    // read params from yaml
    nh.param<std::string>("robot_name", this->robot_name, "");
    this->ReadYaml(this->robot_name);
    for (std::string &observation : this->params.observations)
    {
        // In Gazebo, the coordinate system for angular velocity is in the world coordinate system.
        if (observation == "ang_vel")
        {
            observation = "ang_vel_world";
        }
    }

    // history
    if (this->params.observations_history.size() != 0)
    {
        this->history_obs_buf = ObservationBuffer(1, this->params.num_observations, this->params.observations_history.size());
    }

    // Due to the fact that the robot_state_publisher sorts the joint names alphabetically,
    // the mapping table is established according to the order defined in the YAML file
    std::vector<std::string> sorted_joint_controller_names = this->params.joint_controller_names;
    std::sort(sorted_joint_controller_names.begin(), sorted_joint_controller_names.end());
    for (size_t i = 0; i < this->params.joint_controller_names.size(); ++i)
    {
        this->sorted_to_original_index[sorted_joint_controller_names[i]] = i;
    }
    this->mapped_joint_positions = std::vector<double>(this->params.num_of_dofs, 0.0);
    this->mapped_joint_velocities = std::vector<double>(this->params.num_of_dofs, 0.0);
    this->mapped_joint_efforts = std::vector<double>(this->params.num_of_dofs, 0.0);

    // init
    torch::autograd::GradMode::set_enabled(false);
    this->joint_publishers_commands.resize(this->params.num_of_dofs);
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();
    running_state = STATE_RL_RUNNING;

    // model
    std::string model_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + this->robot_name + "/" + this->params.model_name;
    this->model = torch::jit::load(model_path);

    // publisher
    nh.param<std::string>("ros_namespace", this->ros_namespace, "");
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        // joint need to rename as xxx_joint
        this->joint_publishers[this->params.joint_controller_names[i]] =
            nh.advertise<robot_msgs::MotorCommand>(this->ros_namespace + this->params.joint_controller_names[i] + "/command", 10);
    }

    // subscriber
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Sim::CmdvelCallback, this);
    this->model_state_subscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &RL_Sim::ModelStatesCallback, this);
    this->joint_state_subscriber = nh.subscribe<sensor_msgs::JointState>(this->ros_namespace + "joint_states", 10, &RL_Sim::JointStatesCallback, this);

    // service
    nh.param<std::string>("gazebo_model_name", this->gazebo_model_name, "");
    this->gazebo_set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    this->gazebo_pause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    this->gazebo_unpause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.num_of_dofs);
    this->plot_target_joint_pos.resize(this->params.num_of_dofs);
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;
}

RL_Sim::~RL_Sim()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::GetState(RobotState<double> *state)
{
    if (this->params.framework == "isaacgym")
    {
        state->imu.quaternion[3] = this->pose.orientation.w;
        state->imu.quaternion[0] = this->pose.orientation.x;
        state->imu.quaternion[1] = this->pose.orientation.y;
        state->imu.quaternion[2] = this->pose.orientation.z;
    }
    else if (this->params.framework == "isaacsim")
    {
        state->imu.quaternion[0] = this->pose.orientation.w;
        state->imu.quaternion[1] = this->pose.orientation.x;
        state->imu.quaternion[2] = this->pose.orientation.y;
        state->imu.quaternion[3] = this->pose.orientation.z;
    }

    state->imu.gyroscope[0] = this->vel.angular.x;
    state->imu.gyroscope[1] = this->vel.angular.y;
    state->imu.gyroscope[2] = this->vel.angular.z;

    // state->imu.accelerometer

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        state->motor_state.q[i] = this->mapped_joint_positions[i];
        state->motor_state.dq[i] = this->mapped_joint_velocities[i];
        state->motor_state.tauEst[i] = this->mapped_joint_efforts[i];
    }
}

void RL_Sim::SetCommand(const RobotCommand<double> *command)
{
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->joint_publishers_commands[i].q = command->motor_command.q[i];
        this->joint_publishers_commands[i].dq = command->motor_command.dq[i];
        this->joint_publishers_commands[i].kp = command->motor_command.kp[i];
        this->joint_publishers_commands[i].kd = command->motor_command.kd[i];
        this->joint_publishers_commands[i].tau = command->motor_command.tau[i];
    }

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->joint_publishers[this->params.joint_controller_names[i]].publish(this->joint_publishers_commands[i]);
    }
}

void RL_Sim::RobotControl()
{
    if (this->control.control_state == STATE_RESET_SIMULATION)
    {
        gazebo_msgs::SetModelState set_model_state;
        set_model_state.request.model_state.model_name = this->gazebo_model_name;
        set_model_state.request.model_state.pose.position.z = 1.0;
        set_model_state.request.model_state.reference_frame = "world";
        this->gazebo_set_model_state_client.call(set_model_state);

        this->control.control_state = STATE_WAITING;
    }
    if (this->control.control_state == STATE_TOGGLE_SIMULATION)
    {
        std_srvs::Empty empty;
        if (simulation_running)
        {
            this->gazebo_pause_physics_client.call(empty);
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            this->gazebo_unpause_physics_client.call(empty);
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
        this->control.control_state = STATE_WAITING;
    }
    if (simulation_running)
    {
        this->motiontime++;
        this->GetState(&this->robot_state);
        this->StateController(&this->robot_state, &this->robot_command);
        this->SetCommand(&this->robot_command);
    }
}

void RL_Sim::ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    this->vel = msg->twist[2];
    this->pose = msg->pose[2];
}

void RL_Sim::CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    this->cmd_vel = *msg;
}

void RL_Sim::MapData(const std::vector<double> &source_data, std::vector<double> &target_data)
{
    for (size_t i = 0; i < source_data.size(); ++i)
    {
        target_data[i] = source_data[this->sorted_to_original_index[this->params.joint_controller_names[i]]];
    }
}

void RL_Sim::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    MapData(msg->position, this->mapped_joint_positions);
    MapData(msg->velocity, this->mapped_joint_velocities);
    MapData(msg->effort, this->mapped_joint_efforts);
}

void RL_Sim::RunModel()
{
    if (this->running_state == STATE_RL_RUNNING && simulation_running)
    {
        this->obs.lin_vel = torch::tensor({{this->vel.linear.x, this->vel.linear.y, this->vel.linear.z}});
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        // this->obs.commands = torch::tensor({{this->cmd_vel.linear.x, this->cmd_vel.linear.y, this->cmd_vel.angular.z}});
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

        // this->TorqueProtect(origin_output_torques);

        this->output_torques = torch::clamp(origin_output_torques, -(this->params.torque_limits), this->params.torque_limits);
        this->output_dof_pos = this->ComputePosition(this->obs.actions);

#ifdef CSV_LOGGER
        torch::Tensor tau_est = torch::tensor(this->mapped_joint_efforts).unsqueeze(0);
        this->CSVLogger(this->output_torques, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

torch::Tensor RL_Sim::Forward()
{
    torch::autograd::GradMode::set_enabled(false);

    torch::Tensor clamped_obs = this->ComputeObservation();

    torch::Tensor actions;
    if (this->params.observations_history.size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.observations_history);
        actions = this->model.forward({this->history_obs}).toTensor();
    }
    else
    {
        actions = this->model.forward({clamped_obs}).toTensor();
    }

    if (this->params.clip_actions_upper.numel() != 0 && this->params.clip_actions_lower.numel() != 0)
    {
        return torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);
    }
    else
    {
        return actions;
    }
}

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->mapped_joint_positions[i]);
        this->plot_target_joint_pos[i].push_back(this->joint_publishers_commands[i].q);
        plt::subplot(4, 3, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
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
