#ifndef RL_SDK_HPP
#define RL_SDK_HPP

#include <torch/script.h>
#include <iostream>
#include <string>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

namespace LOGGER {
    const char* const INFO    = "\033[0;37m[INFO]\033[0m ";
    const char* const WARNING = "\033[0;33m[WARNING]\033[0m ";
    const char* const ERROR   = "\033[0;31m[ERROR]\033[0m ";
    const char* const DEBUG   = "\033[0;32m[DEBUG]\033[0m ";
}

template<typename T>
struct RobotCommand
{
    struct MotorCommand
    {
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> tau = std::vector<T>(32, 0.0);
        std::vector<T> kp = std::vector<T>(32, 0.0);
        std::vector<T> kd = std::vector<T>(32, 0.0);
    } motor_command;
};

template<typename T>
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
        std::vector<T> tauEst = std::vector<T>(32, 0.0);
        std::vector<T> cur = std::vector<T>(32, 0.0);
    } motor_state;
};

enum STATE {
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
};

struct ModelParams
{
    std::string model_name;
    std::string framework;
    bool use_history;
    double dt;
    int decimation;
    int num_observations;
    std::vector<std::string> observations;
    double damping;
    double stiffness;
    double action_scale;
    double hip_scale_reduction;
    std::vector<int> hip_scale_reduction_indices;
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
    RL(){};
    ~RL(){};

    ModelParams params;
    Observations obs;

    RobotState<double> robot_state;
    RobotCommand<double> robot_command;

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
    torch::Tensor ComputeTorques(torch::Tensor actions);
    torch::Tensor ComputePosition(torch::Tensor actions);
    torch::Tensor QuatRotateInverse(torch::Tensor q, torch::Tensor v, const std::string& framework);

    // yaml params
    void ReadYaml(std::string robot_name);

    // csv logger
    std::string csv_filename;
    void CSVInit(std::string robot_name);
    void CSVLogger(torch::Tensor torque, torch::Tensor tau_est, torch::Tensor joint_pos, torch::Tensor joint_pos_target, torch::Tensor joint_vel);

    // control
    Control control;
    void KeyboardInterface();

    // others
    std::string robot_name;
    STATE running_state = STATE_RL_RUNNING; // default running_state set to STATE_RL_RUNNING
    bool simulation_running = false;

    // protect func
    void TorqueProtect(torch::Tensor origin_output_torques);

protected:
    // rl module
    torch::jit::script::Module model;
    // output buffer
    torch::Tensor output_torques;
    torch::Tensor output_dof_pos;
};

#endif