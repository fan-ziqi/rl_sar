/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sdk.hpp"

void RL::StateController(const RobotState<float>* state, RobotCommand<float>* command)
{
    auto updateState = [&](std::shared_ptr<FSMState> statePtr)
    {
        if (auto rl_fsm_state = std::dynamic_pointer_cast<RLFSMState>(statePtr))
        {
            rl_fsm_state->fsm_state = state;
            rl_fsm_state->fsm_command = command;
        }
    };
    for (auto& pair : fsm.states_)
    {
        updateState(pair.second);
    }

    fsm.Run();
}

std::vector<float> RL::ComputeObservation()
{
    std::vector<std::vector<float>> obs_list;

    for (const std::string &observation : this->params.observations)
    {
        if (observation == "lin_vel")
        {
            obs_list.push_back(this->obs.lin_vel * this->params.lin_vel_scale);
        }
        else if (observation == "ang_vel_body")
        {
            obs_list.push_back(this->obs.ang_vel * this->params.ang_vel_scale);
        }
        else if (observation == "ang_vel_world")
        {
            obs_list.push_back(this->QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) * this->params.ang_vel_scale);
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(this->QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(this->obs.commands * this->params.commands_scale);
        }
        else if (observation == "dof_pos")
        {
            std::vector<float> dof_pos_rel = this->obs.dof_pos - this->params.default_dof_pos;
            for (int i : this->params.wheel_indices)
            {
                dof_pos_rel[i] = 0.0f;
            }
            obs_list.push_back(dof_pos_rel * this->params.dof_pos_scale);
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(this->obs.dof_vel * this->params.dof_vel_scale);
        }
        else if (observation == "actions")
        {
            obs_list.push_back(this->obs.actions);
        }
        else if (observation == "phase")
        {
            float motion_time = this->episode_length_buf * this->params.dt * this->params.decimation;
            float phase = 3.1415926f * motion_time / 2.0f;
            std::vector<float> phase_tensor = {
                std::sin(phase),
                std::cos(phase),
                std::sin(phase / 2.0f),
                std::cos(phase / 2.0f),
                std::sin(phase / 4.0f),
                std::cos(phase / 4.0f)
            };
            obs_list.push_back(phase_tensor);
        }
        else if (observation == "g1_phase")
        {
            float motion_time = this->episode_length_buf * this->params.dt * this->params.decimation;
            float period = 0.8f;
            float count = motion_time;
            float phase = std::fmod(count, period) / period;
            std::vector<float> phase_tensor = {
                std::sin(2.0f * 3.1415926f * phase),
                std::cos(2.0f * 3.1415926f * phase)
            };
            obs_list.push_back(phase_tensor);
        }
        else if (observation == "g1_mimic_phase")
        {
            float motion_time = this->episode_length_buf * this->params.dt * this->params.decimation;
            float count = motion_time;
            float phase = count / this->motion_length;
            std::vector<float> phase_vec = {phase};
            obs_list.push_back(phase_vec);
        }
    }

    this->obs_dims.clear();
    for (const auto& obs : obs_list)
    {
       this->obs_dims.push_back(obs.size());
    }

    std::vector<float> obs;
    for (const auto& obs_vec : obs_list)
    {
        obs.insert(obs.end(), obs_vec.begin(), obs_vec.end());
    }
    std::vector<float> clamped_obs = clamp(obs, -this->params.clip_obs, this->params.clip_obs);
    return clamped_obs;
}

void RL::InitObservations()
{
    this->obs.lin_vel = {0.0f, 0.0f, 0.0f};
    this->obs.ang_vel = {0.0f, 0.0f, 0.0f};
    this->obs.gravity_vec = {0.0f, 0.0f, -1.0f};
    this->obs.commands = {0.0f, 0.0f, 0.0f};
    this->obs.base_quat = {0.0f, 0.0f, 0.0f, 1.0f};
    this->obs.dof_pos = this->params.default_dof_pos;
    this->obs.dof_vel.resize(this->params.num_of_dofs, 0.0f);
    this->obs.actions.resize(this->params.num_of_dofs, 0.0f);
    this->ComputeObservation();
}

void RL::InitOutputs()
{
    this->output_dof_tau.resize(this->params.num_of_dofs, 0.0f);
    this->output_dof_pos = this->params.default_dof_pos;
    this->output_dof_vel.resize(this->params.num_of_dofs, 0.0f);
}

void RL::InitControl()
{
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
}

void RL::InitJointNum(size_t num_joints)
{
    this->robot_state.motor_state.resize(num_joints);
    this->start_state.motor_state.resize(num_joints);
    this->now_state.motor_state.resize(num_joints);
    this->robot_command.motor_command.resize(num_joints);
}

void RL::InitRL(std::string robot_path)
{
    this->ReadYamlRL(robot_path);
    for (std::string &observation : this->params.observations)
    {
        if (observation == "ang_vel")
        {
            // In ROS1 Gazebo, the coordinate system for angular velocity is in the world coordinate system.
            // In ROS2 Gazebo and real robot, the coordinate system for angular velocity is in the body coordinate system.
            observation = this->ang_vel_type;
        }
    }

    // init joint num first
    this->InitJointNum(this->params.num_of_dofs);

    // init rl
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();

    // init obs history
    if (!this->params.observations_history.empty())
    {
        int history_length = *std::max_element(this->params.observations_history.begin(), this->params.observations_history.end()) + 1;
        this->history_obs_buf = ObservationBuffer(1, this->obs_dims, history_length, this->params.observations_history_priority);
    }

    // init model
    std::string model_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/policy/" + robot_path + "/" + this->params.model_name;
    this->model = InferenceRuntime::ModelFactory::load_model(model_path);
}

void RL::ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau)
{
    std::vector<float> actions_scaled = actions * this->params.action_scale;
    std::vector<float> pos_actions_scaled = actions_scaled;
    std::vector<float> vel_actions_scaled(actions.size(), 0.0f);
    for (int i : this->params.wheel_indices)
    {
        pos_actions_scaled[i] = 0.0f;
        vel_actions_scaled[i] = actions_scaled[i];
    }
    std::vector<float> all_actions_scaled = pos_actions_scaled + vel_actions_scaled;
    output_dof_pos = pos_actions_scaled + this->params.default_dof_pos;
    output_dof_vel = vel_actions_scaled;
    output_dof_tau = this->params.rl_kp * (all_actions_scaled + this->params.default_dof_pos - this->obs.dof_pos) - this->params.rl_kd * this->obs.dof_vel;
    output_dof_tau = clamp(output_dof_tau, -this->params.torque_limits, this->params.torque_limits);
}

std::vector<float> RL::QuatRotateInverse(const std::vector<float>& q, const std::vector<float>& v)
{
    // wxyz
    float q_w = q[0];
    float q_x = q[1];
    float q_y = q[2];
    float q_z = q[3];

    float v_x = v[0];
    float v_y = v[1];
    float v_z = v[2];

    float a_x = v_x * (2.0f * q_w * q_w - 1.0f);
    float a_y = v_y * (2.0f * q_w * q_w - 1.0f);
    float a_z = v_z * (2.0f * q_w * q_w - 1.0f);

    float cross_x = q_y * v_z - q_z * v_y;
    float cross_y = q_z * v_x - q_x * v_z;
    float cross_z = q_x * v_y - q_y * v_x;

    float b_x = cross_x * q_w * 2.0f;
    float b_y = cross_y * q_w * 2.0f;
    float b_z = cross_z * q_w * 2.0f;

    float dot = q_x * v_x + q_y * v_y + q_z * v_z;

    float c_x = q_x * dot * 2.0f;
    float c_y = q_y * dot * 2.0f;
    float c_z = q_z * dot * 2.0f;

    return {a_x - b_x + c_x, a_y - b_y + c_y, a_z - b_z + c_z};
}

void RL::TorqueProtect(const std::vector<float>& origin_output_dof_tau)
{
    std::vector<int> out_of_range_indices;
    std::vector<float> out_of_range_values;
    for (size_t i = 0; i < origin_output_dof_tau.size(); ++i)
    {
        float torque_value = origin_output_dof_tau[i];
        float limit_lower = -this->params.torque_limits[i];
        float limit_upper = this->params.torque_limits[i];

        if (torque_value < limit_lower || torque_value > limit_upper)
        {
            out_of_range_indices.push_back(i);
            out_of_range_values.push_back(torque_value);
        }
    }
    if (!out_of_range_indices.empty())
    {
        for (size_t i = 0; i < out_of_range_indices.size(); ++i)
        {
            int index = out_of_range_indices[i];
            float value = out_of_range_values[i];
            float limit_lower = -this->params.torque_limits[index];
            float limit_upper = this->params.torque_limits[index];

            std::cout << LOGGER::WARNING << "Torque(" << index + 1 << ")=" << value << " out of range(" << limit_lower << ", " << limit_upper << ")" << std::endl;
        }
        // Just a reminder, no protection
        // this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::INFO << "Switching to STATE_POS_GETDOWN"<< std::endl;
    }
}

void RL::AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold)
{
    float rad2deg = 57.2958f;
    float w, x, y, z;

    w = quaternion[0];
    x = quaternion[1];
    y = quaternion[2];
    z = quaternion[3];

    // Calculate roll (rotation around the X-axis)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    float roll = std::atan2(sinr_cosp, cosr_cosp) * rad2deg;

    // Calculate pitch (rotation around the Y-axis)
    float sinp = 2 * (w * y - z * x);
    float pitch;
    if (std::fabs(sinp) >= 1)
    {
        pitch = std::copysign(90.0f, sinp); // Clamp to avoid out-of-range values
    }
    else
    {
        pitch = std::asin(sinp) * rad2deg;
    }

    if (std::fabs(roll) > roll_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Roll exceeds " << roll_threshold << " degrees. Current: " << roll << " degrees." << std::endl;
    }
    if (std::fabs(pitch) > pitch_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Pitch exceeds " << pitch_threshold << " degrees. Current: " << pitch << " degrees." << std::endl;
    }
}

#include <termios.h>
#include <sys/ioctl.h>
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

void RL::KeyboardInterface()
{
    if (kbhit())
    {
        int c = fgetc(stdin);
        switch (c)
        {
        case '0': this->control.SetKeyboard(Input::Keyboard::Num0); break;
        case '1': this->control.SetKeyboard(Input::Keyboard::Num1); break;
        case '2': this->control.SetKeyboard(Input::Keyboard::Num2); break;
        case '3': this->control.SetKeyboard(Input::Keyboard::Num3); break;
        case '4': this->control.SetKeyboard(Input::Keyboard::Num4); break;
        case '5': this->control.SetKeyboard(Input::Keyboard::Num5); break;
        case '6': this->control.SetKeyboard(Input::Keyboard::Num6); break;
        case '7': this->control.SetKeyboard(Input::Keyboard::Num7); break;
        case '8': this->control.SetKeyboard(Input::Keyboard::Num8); break;
        case '9': this->control.SetKeyboard(Input::Keyboard::Num9); break;
        case 'a': case 'A': this->control.SetKeyboard(Input::Keyboard::A); break;
        case 'b': case 'B': this->control.SetKeyboard(Input::Keyboard::B); break;
        case 'c': case 'C': this->control.SetKeyboard(Input::Keyboard::C); break;
        case 'd': case 'D': this->control.SetKeyboard(Input::Keyboard::D); break;
        case 'e': case 'E': this->control.SetKeyboard(Input::Keyboard::E); break;
        case 'f': case 'F': this->control.SetKeyboard(Input::Keyboard::F); break;
        case 'g': case 'G': this->control.SetKeyboard(Input::Keyboard::G); break;
        case 'h': case 'H': this->control.SetKeyboard(Input::Keyboard::H); break;
        case 'i': case 'I': this->control.SetKeyboard(Input::Keyboard::I); break;
        case 'j': case 'J': this->control.SetKeyboard(Input::Keyboard::J); break;
        case 'k': case 'K': this->control.SetKeyboard(Input::Keyboard::K); break;
        case 'l': case 'L': this->control.SetKeyboard(Input::Keyboard::L); break;
        case 'm': case 'M': this->control.SetKeyboard(Input::Keyboard::M); break;
        case 'n': case 'N': this->control.SetKeyboard(Input::Keyboard::N); break;
        case 'o': case 'O': this->control.SetKeyboard(Input::Keyboard::O); break;
        case 'p': case 'P': this->control.SetKeyboard(Input::Keyboard::P); break;
        case 'q': case 'Q': this->control.SetKeyboard(Input::Keyboard::Q); break;
        case 'r': case 'R': this->control.SetKeyboard(Input::Keyboard::R); break;
        case 's': case 'S': this->control.SetKeyboard(Input::Keyboard::S); break;
        case 't': case 'T': this->control.SetKeyboard(Input::Keyboard::T); break;
        case 'u': case 'U': this->control.SetKeyboard(Input::Keyboard::U); break;
        case 'v': case 'V': this->control.SetKeyboard(Input::Keyboard::V); break;
        case 'w': case 'W': this->control.SetKeyboard(Input::Keyboard::W); break;
        case 'x': case 'X': this->control.SetKeyboard(Input::Keyboard::X); break;
        case 'y': case 'Y': this->control.SetKeyboard(Input::Keyboard::Y); break;
        case 'z': case 'Z': this->control.SetKeyboard(Input::Keyboard::Z); break;
        case ' ': this->control.SetKeyboard(Input::Keyboard::Space); break;
        case '\n': case '\r': this->control.SetKeyboard(Input::Keyboard::Enter); break;
        case 27: this->control.SetKeyboard(Input::Keyboard::Escape); break;
        case 0xE0:
        {
            int ext = fgetc(stdin);
            switch (ext)
            {
            case 72: this->control.SetKeyboard(Input::Keyboard::Up); break;
            case 80: this->control.SetKeyboard(Input::Keyboard::Down); break;
            case 75: this->control.SetKeyboard(Input::Keyboard::Left); break;
            case 77: this->control.SetKeyboard(Input::Keyboard::Right); break;
            default:  break;
            }
        } break;
        default:  break;
        }
    }
}

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node &node)
{
    std::vector<T> values;
    for (const auto &val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

void RL::ReadYamlBase(std::string robot_path)
{
    // The config file is located at "rl_sar/src/rl_sar/policy/<robot_path>/base.yaml"
    std::string config_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/policy/" + robot_path + "/base.yaml";
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[robot_path];
    }
    catch (YAML::BadFile &e)
    {
        std::cout << LOGGER::ERROR << "The file '" << config_path << "' does not exist" << std::endl;
        return;
    }

    this->params.dt = config["dt"].as<float>();
    this->params.decimation = config["decimation"].as<int>();
    this->params.wheel_indices = ReadVectorFromYaml<int>(config["wheel_indices"]);
    this->params.num_of_dofs = config["num_of_dofs"].as<int>();
    this->params.fixed_kp = ReadVectorFromYaml<float>(config["fixed_kp"]);
    this->params.fixed_kd = ReadVectorFromYaml<float>(config["fixed_kd"]);
    this->params.torque_limits = ReadVectorFromYaml<float>(config["torque_limits"]);
    this->params.default_dof_pos = ReadVectorFromYaml<float>(config["default_dof_pos"]);
    this->params.joint_names = ReadVectorFromYaml<std::string>(config["joint_names"]);
    this->params.joint_controller_names = ReadVectorFromYaml<std::string>(config["joint_controller_names"]);
    this->params.joint_mapping = ReadVectorFromYaml<int>(config["joint_mapping"]);
}

void RL::ReadYamlRL(std::string robot_path)
{
    // The config file is located at "rl_sar/src/rl_sar/policy/<robot_path>/config.yaml"
    std::string config_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/policy/" + robot_path + "/config.yaml";
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[robot_path];
    }
    catch (YAML::BadFile &e)
    {
        std::cout << LOGGER::ERROR << "The file '" << config_path << "' does not exist" << std::endl;
        return;
    }

    this->params.model_name = config["model_name"].as<std::string>();
    this->params.num_observations = config["num_observations"].as<int>();
    this->params.observations = ReadVectorFromYaml<std::string>(config["observations"]);
    if (config["observations_history"].IsNull())
    {
        this->params.observations_history = {};
    }
    else
    {
        this->params.observations_history = ReadVectorFromYaml<int>(config["observations_history"]);
    }
    this->params.observations_history_priority = config["observations_history_priority"].as<std::string>();
    this->params.clip_obs = config["clip_obs"].as<float>();
    if (config["clip_actions_lower"].IsNull() && config["clip_actions_upper"].IsNull())
    {
        this->params.clip_actions_upper = {};
        this->params.clip_actions_lower = {};
    }
    else
    {
        this->params.clip_actions_upper = ReadVectorFromYaml<float>(config["clip_actions_upper"]);
        this->params.clip_actions_lower = ReadVectorFromYaml<float>(config["clip_actions_lower"]);
    }
    this->params.action_scale = ReadVectorFromYaml<float>(config["action_scale"]);
    this->params.wheel_indices = ReadVectorFromYaml<int>(config["wheel_indices"]);
    this->params.num_of_dofs = config["num_of_dofs"].as<int>();
    this->params.lin_vel_scale = config["lin_vel_scale"].as<float>();
    this->params.ang_vel_scale = config["ang_vel_scale"].as<float>();
    this->params.dof_pos_scale = config["dof_pos_scale"].as<float>();
    this->params.dof_vel_scale = config["dof_vel_scale"].as<float>();
    this->params.commands_scale = ReadVectorFromYaml<float>(config["commands_scale"]);
    // this->params.commands_scale = {this->params.lin_vel_scale, this->params.lin_vel_scale, this->params.ang_vel_scale};
    this->params.rl_kp = ReadVectorFromYaml<float>(config["rl_kp"]);
    this->params.rl_kd = ReadVectorFromYaml<float>(config["rl_kd"]);
    this->params.fixed_kp = ReadVectorFromYaml<float>(config["fixed_kp"]);
    this->params.fixed_kd = ReadVectorFromYaml<float>(config["fixed_kd"]);
    this->params.torque_limits = ReadVectorFromYaml<float>(config["torque_limits"]);
    this->params.default_dof_pos = ReadVectorFromYaml<float>(config["default_dof_pos"]);
    this->params.joint_mapping = ReadVectorFromYaml<int>(config["joint_mapping"]);
}

void RL::CSVInit(std::string robot_path)
{
    csv_filename = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/policy/" + robot_path + "/motor";

    // Uncomment these lines if need timestamp for file name
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // std::stringstream ss;
    // ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    // std::string timestamp = ss.str();
    // csv_filename += "_" + timestamp;

    csv_filename += ".csv";
    std::ofstream file(csv_filename.c_str());

    for(int i = 0; i < this->params.num_of_dofs; ++i) { file << "tau_cal_" << i << ","; }
    for(int i = 0; i < this->params.num_of_dofs; ++i) { file << "tau_est_" << i << ","; }
    for(int i = 0; i < this->params.num_of_dofs; ++i) { file << "joint_pos_" << i << ","; }
    for(int i = 0; i < this->params.num_of_dofs; ++i) { file << "joint_pos_target_" << i << ","; }
    for(int i = 0; i < this->params.num_of_dofs; ++i) { file << "joint_vel_" << i << ","; }

    file << std::endl;

    file.close();
}

void RL::CSVLogger(const std::vector<float>& torque, const std::vector<float>& tau_est, const std::vector<float>& joint_pos, const std::vector<float>& joint_pos_target, const std::vector<float>& joint_vel)
{
    std::ofstream file(csv_filename.c_str(), std::ios_base::app);

    for(int i = 0; i < this->params.num_of_dofs; ++i) { file << torque[i] << ","; }
    for(int i = 0; i < this->params.num_of_dofs; ++i) { file << tau_est[i] << ","; }
    for(int i = 0; i < this->params.num_of_dofs; ++i) { file << joint_pos[i] << ","; }
    for(int i = 0; i < this->params.num_of_dofs; ++i) { file << joint_pos_target[i] << ","; }
    for(int i = 0; i < this->params.num_of_dofs; ++i) { file << joint_vel[i] << ","; }

    file << std::endl;

    file.close();
}
