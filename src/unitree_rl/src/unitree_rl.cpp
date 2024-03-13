#include "../include/unitree_rl.hpp"
#include <ros/package.h>

Unitree_RL::Unitree_RL()
{
    ros::NodeHandle nh;
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
    this->params.default_dof_pos = torch::tensor({{0.1000, 0.8000, -1.5000,   // front left
                                                  -0.1000, 0.8000, -1.5000,   // front right
                                                   0.1000, 1.0000, -1.5000,   // rear  left
                                                  -0.1000, 1.0000, -1.5000}});// rear  right

    this->history_obs_buf = ObservationBuffer(1, this->params.num_observations, 6);

    cmd_vel_subscriber_ = nh.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 10, &Unitree_RL::cmdvelCallback, this);

    timer = nh.createTimer(ros::Duration(0.005), &Unitree_RL::runModel, this);

    ros_namespace = "/a1_gazebo/";

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
        "/gazebo/model_states", 10, &Unitree_RL::modelStatesCallback, this);

    joint_state_subscriber_ = nh.subscribe<sensor_msgs::JointState>(
        "/a1_gazebo/joint_states", 10, &Unitree_RL::jointStatesCallback, this);
}

void Unitree_RL::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{

    vel = msg->twist[2];
    pose = msg->pose[2];
}

void Unitree_RL::cmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel = *msg;
}

void Unitree_RL::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_positions = msg->position;
    joint_velocities = msg->velocity;
}

void Unitree_RL::runModel(const ros::TimerEvent &event)
{
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    // std::cout << "Execution time: " << duration << " microseconds" << std::endl;
    start_time = std::chrono::high_resolution_clock::now();

    this->obs.lin_vel = torch::tensor({{vel.linear.x, vel.linear.y, vel.linear.z}});
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

    torch::Tensor actions = this->forward();
    torques = this->compute_torques(actions);

    for (int i = 0; i < 12; ++i)
    {
        torque_commands[i].tau = torques[0][i].item<double>();
        torque_commands[i].mode = 0x0A;

        torque_publishers[joint_names[i]].publish(torque_commands[i]);
    }
}

torch::Tensor Unitree_RL::compute_observation()
{
    torch::Tensor obs = torch::cat({// (this->quat_rotate_inverse(this->base_quat, this->lin_vel)) * this->params.lin_vel_scale,
                                    (this->quat_rotate_inverse(this->obs.base_quat, this->obs.ang_vel)) * this->params.ang_vel_scale,
                                    this->quat_rotate_inverse(this->obs.base_quat, this->obs.gravity_vec),
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
    ros::init(argc, argv, "unitree_rl");
    Unitree_RL unitree_rl;
    ros::spin();
    return 0;
}
