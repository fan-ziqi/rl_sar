/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_SDK_HPP
#define RL_SDK_HPP

#include <iostream>
#include <string>
#include <exception>
#include <unistd.h>
#include <algorithm>
#include <tbb/concurrent_queue.h>
#include <vector>
#include <memory>
#include <fstream>

#include <yaml-cpp/yaml.h>
#include "fsm_core.hpp"
#include "observation_buffer.hpp"
#include "vector_math.hpp"
#include "model_interface.hpp"

namespace LOGGER
{
    const char *const INFO    = "\033[0;37m[INFO]\033[0m ";
    const char *const WARNING = "\033[0;33m[WARNING]\033[0m ";
    const char *const ERROR   = "\033[0;31m[ERROR]\033[0m ";
    const char *const DEBUG   = "\033[0;32m[DEBUG]\033[0m ";
    const char *const NOTE    = "\033[0;34m[NOTE]\033[0m ";
}

template <typename T>
struct RobotCommand
{
    struct MotorCommand
    {
        std::vector<int> mode;
        std::vector<T> q;
        std::vector<T> dq;
        std::vector<T> tau;
        std::vector<T> kp;
        std::vector<T> kd;

        // 初始化方法
        void resize(size_t num_joints)
        {
            mode.resize(num_joints, 0);
            q.resize(num_joints, 0.0f);
            dq.resize(num_joints, 0.0f);
            tau.resize(num_joints, 0.0f);
            kp.resize(num_joints, 0.0f);
            kd.resize(num_joints, 0.0f);
        }
    } motor_command;
};

template <typename T>
struct RobotState
{
    struct IMU
    {
        std::vector<T> quaternion = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
        std::vector<T> gyroscope = {0.0f, 0.0f, 0.0f};
        std::vector<T> accelerometer = {0.0f, 0.0f, 0.0f};
    } imu;

    struct MotorState
    {
        std::vector<T> q;
        std::vector<T> dq;
        std::vector<T> ddq;
        std::vector<T> tau_est;
        std::vector<T> cur;

        // 初始化方法
        void resize(size_t num_joints)
        {
            q.resize(num_joints, 0.0f);
            dq.resize(num_joints, 0.0f);
            ddq.resize(num_joints, 0.0f);
            tau_est.resize(num_joints, 0.0f);
            cur.resize(num_joints, 0.0f);
        }
    } motor_state;
};

namespace Input
{
    // Recommend: Num0-GetUp Num9-GetDown N-ToggleNavMode
    //            R-SimReset Enter-SimToggle
    //            M-MotorEnable K-MotorDisable P-MotorPassive
    //            Num1-BaseLocomotion Num2-Num8-Skills(7)
    //            WS-AxisX AD-AxisY QE-AxisYaw Space-AxisClear
    enum class Keyboard
    {
        None = 0,
        A, B, C, D, E, F, G, H, I, J, K, L, M,
        N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
        Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
        Space, Enter, Escape,
        Up, Down, Left, Right
    };

    // Recommend: A-GetUp B-GetDown X-ToggleNavMode Y-None
    //            RB_Y-SimReset RB_X-SimToggle
    //            LB_A-MotorEnable LB_B-MotorDisable LB_X-MotorPassive
    //            RB_DPadUp-BaseLocomotion RB_DPadOthers/LB_DPadOthers-Skills(7)
    //            LY-AxisX LX-AxisY RX-AxisYaw
    enum class Gamepad
    {
        None = 0,
        A, B, X, Y, LB, RB, LStick, RStick, DPadUp, DPadDown, DPadLeft, DPadRight,
        LB_A, LB_B, LB_X, LB_Y, LB_LStick, LB_RStick, LB_DPadUp, LB_DPadDown, LB_DPadLeft, LB_DPadRight,
        RB_A, RB_B, RB_X, RB_Y, RB_LStick, RB_RStick, RB_DPadUp, RB_DPadDown, RB_DPadLeft, RB_DPadRight,
        LB_RB
    };
}

struct Control
{
    Input::Keyboard current_keyboard = Input::Keyboard::None, last_keyboard = Input::Keyboard::None;
    Input::Gamepad current_gamepad = Input::Gamepad::None, last_gamepad = Input::Gamepad::None;

    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    bool navigation_mode = false;

    void SetKeyboard(Input::Keyboard keyboad)
    {
        if (current_keyboard != keyboad)
        {
            last_keyboard = current_keyboard;
            current_keyboard = keyboad;
        }
    }

    void SetGamepad(Input::Gamepad gamepad)
    {
        if (current_gamepad != gamepad)
        {
            last_gamepad = current_gamepad;
            current_gamepad = gamepad;
        }
    }
};

struct ModelParams
{
    std::string model_name;
    float dt;
    int decimation;
    int num_observations;
    std::vector<std::string> observations;
    std::vector<int> observations_history;
    std::string observations_history_priority;
    float damping;
    float stiffness;
    std::vector<float> action_scale;
    std::vector<int> wheel_indices;
    int num_of_dofs;
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float clip_obs;
    std::vector<float> clip_actions_upper;
    std::vector<float> clip_actions_lower;
    std::vector<float> torque_limits;
    std::vector<float> rl_kd;
    std::vector<float> rl_kp;
    std::vector<float> fixed_kp;
    std::vector<float> fixed_kd;
    std::vector<float> commands_scale;
    std::vector<float> default_dof_pos;
    std::vector<std::string> joint_controller_names;
    std::vector<std::string> joint_names;
    std::vector<int> joint_mapping;
};

template <typename T>
struct Observations
{
    std::vector<T> lin_vel;
    std::vector<T> ang_vel;
    std::vector<T> gravity_vec;
    std::vector<T> commands;
    std::vector<T> base_quat;
    std::vector<T> dof_pos;
    std::vector<T> dof_vel;
    std::vector<T> actions;
};

class RL
{
public:
    RL() {};
    ~RL() {};

    ModelParams params;
    Observations<float> obs;
    std::vector<int> obs_dims;

    RobotState<float> robot_state;
    RobotCommand<float> robot_command;
    tbb::concurrent_queue<std::vector<float>> output_dof_pos_queue;
    tbb::concurrent_queue<std::vector<float>> output_dof_vel_queue;
    tbb::concurrent_queue<std::vector<float>> output_dof_tau_queue;

    FSM fsm;
    RobotState<float> start_state;
    RobotState<float> now_state;
    float running_percent = 0.0f;
    bool rl_init_done = false;

    // init
    void InitObservations();
    void InitOutputs();
    void InitControl();
    void InitRL(std::string robot_path);
    void InitJointNum(size_t num_joints);

    // rl functions
    virtual std::vector<float> Forward() = 0;
    std::vector<float> ComputeObservation();
    virtual void GetState(RobotState<float> *state) = 0;
    virtual void SetCommand(const RobotCommand<float> *command) = 0;
    void StateController(const RobotState<float> *state, RobotCommand<float> *command);
    void ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau);
    std::vector<float> QuatRotateInverse(const std::vector<float> &q, const std::vector<float> &v);

    // yaml params
    void ReadYamlBase(std::string robot_name);
    void ReadYamlRL(std::string robot_name);

    // csv logger
    std::string csv_filename;
    void CSVInit(std::string robot_name);
    void CSVLogger(const std::vector<float> &torque, const std::vector<float> &tau_est, const std::vector<float> &joint_pos, const std::vector<float> &joint_pos_target, const std::vector<float> &joint_vel);

    // control
    Control control;
    void KeyboardInterface();

    // history buffer
    ObservationBuffer history_obs_buf;
    std::vector<float> history_obs;

    // others
    std::string robot_name, config_name;
    bool simulation_running = true;
    std::string ang_vel_type = "ang_vel_body";  // "ang_vel_world" or "ang_vel_body"
    unsigned long long episode_length_buf = 0;
    float motion_length = 0.0;

    // protect func
    void TorqueProtect(const std::vector<float> &origin_output_dof_tau);
    void AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold);

    // rl module
    std::unique_ptr<ModelInterface::Model> model;
    // output buffer
    std::vector<float> output_dof_tau;
    std::vector<float> output_dof_pos;
    std::vector<float> output_dof_vel;
};

class RLFSMState : public FSMState
{
public:
    RLFSMState(RL& rl, const std::string& name)
        : FSMState(name), rl(rl), fsm_state(nullptr), fsm_command(nullptr) {}
    RL& rl;
    const RobotState<float> *fsm_state;
    RobotCommand<float> *fsm_command;
};

#endif // RL_SDK_HPP
