/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_SDK_HPP
#define RL_SDK_HPP

#include <torch/script.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <tbb/concurrent_queue.h>

#include <yaml-cpp/yaml.h>

namespace LOGGER
{
    const char *const INFO    = "\033[0;37m[INFO]\033[0m ";
    const char *const WARNING = "\033[0;33m[WARNING]\033[0m ";
    const char *const ERROR   = "\033[0;31m[ERROR]\033[0m ";
    const char *const DEBUG   = "\033[0;32m[DEBUG]\033[0m ";
}

template <typename T>
struct RobotCommand
{
    struct MotorCommand
    {
        std::vector<int> mode = std::vector<int>(32, 0);
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> tau = std::vector<T>(32, 0.0);
        std::vector<T> kp = std::vector<T>(32, 0.0);
        std::vector<T> kd = std::vector<T>(32, 0.0);
    } motor_command;
};

template <typename T>
struct RobotState
{
    struct IMU
    {
        std::vector<T> quaternion = {1.0, 0.0, 0.0, 0.0}; // w, x, y, z
        std::vector<T> gyroscope = {0.0, 0.0, 0.0};
        std::vector<T> accelerometer = {0.0, 0.0, 0.0};
    } imu;

    struct MotorState
    {
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> ddq = std::vector<T>(32, 0.0);
        std::vector<T> tau_est = std::vector<T>(32, 0.0);
        std::vector<T> cur = std::vector<T>(32, 0.0);
    } motor_state;
};

enum STATE
{
    STATE_WAITING = 0,
    STATE_POS_GETUP,
    STATE_RL_INIT,
    STATE_RL_RUNNING,
    STATE_POS_GETDOWN,
    STATE_RESET_SIMULATION,
    STATE_TOGGLE_SIMULATION,
};

struct Control
{
    STATE control_state;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double wheel = 0.0;
};

struct ModelParams
{
    std::string model_name;
    std::string framework;
    double dt;
    int decimation;
    int num_observations;
    std::vector<std::string> observations;
    std::vector<int> observations_history;
    double damping;
    double stiffness;
    torch::Tensor action_scale;
    std::vector<int> wheel_indices;
    int num_of_dofs;
    double lin_vel_scale;
    double ang_vel_scale;
    double dof_pos_scale;
    double dof_vel_scale;
    double clip_obs;
    torch::Tensor clip_actions_upper;
    torch::Tensor clip_actions_lower;
    torch::Tensor torque_limits;
    torch::Tensor rl_kd;
    torch::Tensor rl_kp;
    torch::Tensor fixed_kp;
    torch::Tensor fixed_kd;
    torch::Tensor commands_scale;
    torch::Tensor default_dof_pos;
    std::vector<std::string> joint_controller_names;
    std::vector<int> command_mapping;
    std::vector<int> state_mapping;
};

struct Observations
{
    torch::Tensor lin_vel;
    torch::Tensor ang_vel;
    torch::Tensor gravity_vec;
    torch::Tensor commands;
    torch::Tensor base_quat;
    torch::Tensor dof_pos;
    torch::Tensor dof_vel;
    torch::Tensor actions;
};

class RL
{
public:
    RL() {};
    ~RL() {};

    ModelParams params;
    Observations obs;

    RobotState<double> robot_state;
    RobotCommand<double> robot_command;
    tbb::concurrent_queue<torch::Tensor> output_dof_pos_queue;
    tbb::concurrent_queue<torch::Tensor> output_dof_vel_queue;
    tbb::concurrent_queue<torch::Tensor> output_dof_tau_queue;

    // init
    void InitObservations();
    void InitOutputs();
    void InitControl();

    // rl functions
    virtual torch::Tensor Forward() = 0;
    torch::Tensor ComputeObservation();
    virtual void GetState(RobotState<double> *state) = 0;
    virtual void SetCommand(const RobotCommand<double> *command) = 0;
    void StateController(const RobotState<double> *state, RobotCommand<double> *command);
    void ComputeOutput(const torch::Tensor &actions, torch::Tensor &output_dof_pos, torch::Tensor &output_dof_vel, torch::Tensor &output_dof_tau);
    torch::Tensor QuatRotateInverse(torch::Tensor q, torch::Tensor v, const std::string &framework);

    // yaml params
    void ReadYaml(std::string robot_path);

    // csv logger
    std::string csv_filename;
    void CSVInit(std::string robot_name);
    void CSVLogger(torch::Tensor torque, torch::Tensor tau_est, torch::Tensor joint_pos, torch::Tensor joint_pos_target, torch::Tensor joint_vel);

    // control
    Control control;
    void KeyboardInterface();

    // others
    std::string robot_name, config_name;
    STATE running_state = STATE_RL_RUNNING; // default running_state set to STATE_RL_RUNNING
    bool simulation_running = false;
    unsigned long long episode_length_buf = 0;

    // protect func
    void TorqueProtect(torch::Tensor origin_output_dof_tau);
    void AttitudeProtect(const std::vector<double> &quaternion, float pitch_threshold, float roll_threshold);

protected:
    // rl module
    torch::jit::script::Module model;
    // output buffer
    torch::Tensor output_dof_tau;
    torch::Tensor output_dof_pos;
    torch::Tensor output_dof_vel;
};

template <typename T>
T clamp(T value, T min, T max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

#endif // RL_SDK_HPP
