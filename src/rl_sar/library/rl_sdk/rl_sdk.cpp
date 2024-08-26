#include "rl_sdk.hpp"

/* You may need to override this Forward() function
torch::Tensor RL_XXX::Forward()
{
    torch::autograd::GradMode::set_enabled(false);
    torch::Tensor clamped_obs = this->ComputeObservation();
    torch::Tensor actions = this->model.forward({clamped_obs}).toTensor();
    torch::Tensor clamped_actions = torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);
    return clamped_actions;
}
*/

torch::Tensor RL::ComputeObservation()
{
    std::vector<torch::Tensor> obs_list;

    for(const std::string& observation : this->params.observations)
    {
        if(observation == "lin_vel")
        {
            obs_list.push_back(this->obs.lin_vel * this->params.lin_vel_scale);
        }
        else if(observation == "ang_vel")
        {
            // obs_list.push_back(this->obs.ang_vel * this->params.ang_vel_scale); // TODO is QuatRotateInverse necessery?
            obs_list.push_back(this->QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel, this->params.framework) * this->params.ang_vel_scale);
        }
        else if(observation == "gravity_vec")
        {
            obs_list.push_back(this->QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec, this->params.framework));
        }
        else if(observation == "commands")
        {
            obs_list.push_back(this->obs.commands * this->params.commands_scale);
        }
        else if(observation == "dof_pos")
        {
            obs_list.push_back((this->obs.dof_pos - this->params.default_dof_pos) * this->params.dof_pos_scale);
        }
        else if(observation == "dof_vel")
        {
            obs_list.push_back(this->obs.dof_vel * this->params.dof_vel_scale);
        }
        else if(observation == "actions")
        {
            obs_list.push_back(this->obs.actions);
        }
    }

    torch::Tensor obs = torch::cat(obs_list, 1);
    torch::Tensor clamped_obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);
    return clamped_obs;
}

void RL::InitObservations()
{
    this->obs.lin_vel = torch::tensor({{0.0, 0.0, 0.0}});
    this->obs.ang_vel = torch::tensor({{0.0, 0.0, 0.0}});
    this->obs.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    this->obs.commands = torch::tensor({{0.0, 0.0, 0.0}});
    this->obs.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    this->obs.dof_pos = this->params.default_dof_pos;
    this->obs.dof_vel = torch::zeros({1, this->params.num_of_dofs});
    this->obs.actions = torch::zeros({1, this->params.num_of_dofs});
}

void RL::InitOutputs()
{
    this->output_torques = torch::zeros({1, this->params.num_of_dofs});
    this->output_dof_pos = this->params.default_dof_pos;
}

void RL::InitControl()
{
    this->control.control_state = STATE_WAITING;
    this->control.x = 0.0;
    this->control.y = 0.0;
    this->control.yaw = 0.0;
}

torch::Tensor RL::ComputeTorques(torch::Tensor actions)
{
    torch::Tensor actions_scaled = actions * this->params.action_scale;
    torch::Tensor output_torques = this->params.rl_kp * (actions_scaled + this->params.default_dof_pos - this->obs.dof_pos) - this->params.rl_kd * this->obs.dof_vel;
    return output_torques;
}

torch::Tensor RL::ComputePosition(torch::Tensor actions)
{
    torch::Tensor actions_scaled = actions * this->params.action_scale;
    return actions_scaled + this->params.default_dof_pos;
}

torch::Tensor RL::QuatRotateInverse(torch::Tensor q, torch::Tensor v, const std::string& framework)
{
    torch::Tensor q_w;
    torch::Tensor q_vec;
    if(framework == "isaacsim")
    {
        q_w = q.index({torch::indexing::Slice(), 0});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(1, 4)});
    }
    else if(framework == "isaacgym")
    {
        q_w = q.index({torch::indexing::Slice(), 3});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
    }
    c10::IntArrayRef shape = q.sizes();
    
    torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
    torch::Tensor b = torch::cross(q_vec, v, -1) * q_w.unsqueeze(-1) * 2.0;
    torch::Tensor c = q_vec * torch::bmm(q_vec.view({shape[0], 1, 3}), v.view({shape[0], 3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

void RL::StateController(const RobotState<double> *state, RobotCommand<double> *command)
{
    static RobotState<double> start_state;
    static RobotState<double> now_state;
    static float getup_percent = 0.0;
    static float getdown_percent = 0.0;

    // waiting
    if(this->running_state == STATE_WAITING)
    {
        for(int i = 0; i < this->params.num_of_dofs; ++i)
        {
            command->motor_command.q[i] = state->motor_state.q[i];
        }
        if(this->control.control_state == STATE_POS_GETUP)
        {
            this->control.control_state = STATE_WAITING;
            getup_percent = 0.0;
            for(int i = 0; i < this->params.num_of_dofs; ++i)
            {
                now_state.motor_state.q[i] = state->motor_state.q[i];
                start_state.motor_state.q[i] = now_state.motor_state.q[i];
            }
            this->running_state = STATE_POS_GETUP;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_POS_GETUP" << std::endl;
        }
    }
    // stand up (position control)
    else if(this->running_state == STATE_POS_GETUP)
    {
        if(getup_percent < 1.0)
        {
            getup_percent += 1 / 500.0;
            getup_percent = getup_percent > 1.0 ? 1.0 : getup_percent;
            for(int i = 0; i < this->params.num_of_dofs; ++i)
            {
                command->motor_command.q[i] = (1 - getup_percent) * now_state.motor_state.q[i] + getup_percent * this->params.default_dof_pos[0][i].item<double>();
                command->motor_command.dq[i] = 0;
                command->motor_command.kp[i] = this->params.fixed_kp[0][i].item<double>();
                command->motor_command.kd[i] = this->params.fixed_kd[0][i].item<double>();
                command->motor_command.tau[i] = 0;
            }
            std::cout << "\r" << std::flush << LOGGER::INFO << "Getting up " << std::fixed << std::setprecision(2) << getup_percent * 100.0 << std::flush;
        }
        if(this->control.control_state == STATE_RL_INIT)
        {
            this->control.control_state = STATE_WAITING;
            this->running_state = STATE_RL_INIT;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_RL_INIT" << std::endl;
        }
        else if(this->control.control_state == STATE_POS_GETDOWN)
        {
            this->control.control_state = STATE_WAITING;
            getdown_percent = 0.0;
            for(int i = 0; i < this->params.num_of_dofs; ++i)
            {
                now_state.motor_state.q[i] = state->motor_state.q[i];
            }
            this->running_state = STATE_POS_GETDOWN;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_POS_GETDOWN" << std::endl;
        }
    }
    // init obs and start rl loop
    else if(this->running_state == STATE_RL_INIT)
    {
        if(getup_percent == 1)
        {
            this->InitObservations();
            this->InitOutputs();
            this->InitControl();
            this->running_state = STATE_RL_RUNNING;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_RL_RUNNING" << std::endl;
        }
    }
    // rl loop
    else if(this->running_state == STATE_RL_RUNNING)
    {
        std::cout << "\r" << std::flush << LOGGER::INFO << "RL Controller x:" << this->control.x << " y:" << this->control.y << " yaw:" << this->control.yaw << std::flush;
        for(int i = 0; i < this->params.num_of_dofs; ++i)
        {
            command->motor_command.q[i] = this->output_dof_pos[0][i].item<double>();
            command->motor_command.dq[i] = 0;
            command->motor_command.kp[i] = this->params.rl_kp[0][i].item<double>();
            command->motor_command.kd[i] = this->params.rl_kd[0][i].item<double>();
            command->motor_command.tau[i] = 0;
        }
        if(this->control.control_state == STATE_POS_GETDOWN)
        {
            this->control.control_state = STATE_WAITING;
            getdown_percent = 0.0;
            for(int i = 0; i < this->params.num_of_dofs; ++i)
            {
                now_state.motor_state.q[i] = state->motor_state.q[i];
            }
            this->running_state = STATE_POS_GETDOWN;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_POS_GETDOWN" << std::endl;
        }
        else if(this->control.control_state == STATE_POS_GETUP)
        {
            this->control.control_state = STATE_WAITING;
            getup_percent = 0.0;
            for(int i = 0; i < this->params.num_of_dofs; ++i)
            {
                now_state.motor_state.q[i] = state->motor_state.q[i];
            }
            this->running_state = STATE_POS_GETUP;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_POS_GETUP" << std::endl;
        }
    }
    // get down (position control)
    else if(this->running_state == STATE_POS_GETDOWN)
    {
        if(getdown_percent < 1.0)
        {
            getdown_percent += 1 / 500.0;
            getdown_percent = getdown_percent > 1.0 ? 1.0 : getdown_percent;
            for(int i = 0; i < this->params.num_of_dofs; ++i)
            {
                command->motor_command.q[i] = (1 - getdown_percent) * now_state.motor_state.q[i] + getdown_percent * start_state.motor_state.q[i];
                command->motor_command.dq[i] = 0;
                command->motor_command.kp[i] = this->params.fixed_kp[0][i].item<double>();
                command->motor_command.kd[i] = this->params.fixed_kd[0][i].item<double>();
                command->motor_command.tau[i] = 0;
            }
            std::cout << "\r" << std::flush << LOGGER::INFO << "Getting down " << std::fixed << std::setprecision(2) << getdown_percent * 100.0 << std::flush;
        }
        if(getdown_percent == 1)
        {
            this->InitObservations();
            this->InitOutputs();
            this->InitControl();
            this->running_state = STATE_WAITING;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_WAITING" << std::endl;
        }
    }
}

void RL::TorqueProtect(torch::Tensor origin_output_torques)
{
    std::vector<int> out_of_range_indices;
    std::vector<double> out_of_range_values;
    for(int i = 0; i < origin_output_torques.size(1); ++i)
    {
        double torque_value = origin_output_torques[0][i].item<double>();
        double limit_lower = -this->params.torque_limits[0][i].item<double>();
        double limit_upper = this->params.torque_limits[0][i].item<double>();

        if(torque_value < limit_lower || torque_value > limit_upper)
        {
            out_of_range_indices.push_back(i);
            out_of_range_values.push_back(torque_value);
        }
    }
    if(!out_of_range_indices.empty())
    {
        for(int i = 0; i < out_of_range_indices.size(); ++i)
        {
            int index = out_of_range_indices[i];
            double value = out_of_range_values[i];
            double limit_lower = -this->params.torque_limits[0][index].item<double>();
            double limit_upper = this->params.torque_limits[0][index].item<double>();

            std::cout << LOGGER::WARNING << "Torque(" << index+1 << ")=" << value << " out of range(" << limit_lower << ", " << limit_upper << ")" << std::endl;
        }
        // Just a reminder, no protection
        // this->control.control_state = STATE_POS_GETDOWN;
        // std::cout << LOGGER::INFO << "Switching to STATE_POS_GETDOWN"<< std::endl;
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
    if(kbhit())
    {
        int c = fgetc(stdin);
        switch(c)
        {
            case '0': this->control.control_state = STATE_POS_GETUP; break;
            case 'p': this->control.control_state = STATE_RL_INIT; break;
            case '1': this->control.control_state = STATE_POS_GETDOWN; break;
            case 'q': break;
            case 'w': this->control.x += 0.1; break;
            case 's': this->control.x -= 0.1; break;
            case 'a': this->control.yaw += 0.1; break;
            case 'd': this->control.yaw -= 0.1; break;
            case 'i': break;
            case 'k': break;
            case 'j': this->control.y += 0.1; break;
            case 'l': this->control.y -= 0.1; break;
            case ' ': this->control.x = 0; this->control.y = 0; this->control.yaw = 0; break;
            case 'r': this->control.control_state = STATE_RESET_SIMULATION; break;
            case '\n': this->control.control_state = STATE_TOGGLE_SIMULATION; break;
            default: break;
        }
    }
}

template<typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node)
{
    std::vector<T> values;
    for(const auto& val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

template<typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node, const std::string& framework, const int& rows, const int& cols)
{
    std::vector<T> values;
    for(const auto& val : node)
    {
        values.push_back(val.as<T>());
    }

    if(framework == "isaacsim")
    {
        std::vector<T> transposed_values(cols * rows);
        for(int r = 0; r < rows; ++r)
        {
            for(int c = 0; c < cols; ++c)
            {
                transposed_values[c * rows + r] = values[r * cols + c];
            }
        }
        return transposed_values;
    }
    else if(framework == "isaacgym")
    {
        return values;
    }
    else
    {
        throw std::invalid_argument("Unsupported framework: " + framework);
    }
}

void RL::ReadYaml(std::string robot_name)
{
    // The config file is located at "rl_sar/src/rl_sar/models/<robot_name>/config.yaml"
    std::string config_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + robot_name + "/config.yaml";
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[robot_name];
    } catch(YAML::BadFile &e)
    {
        std::cout << LOGGER::ERROR << "The file '" << config_path << "' does not exist" << std::endl;
        return;
    }

    this->params.model_name = config["model_name"].as<std::string>();
    this->params.framework = config["framework"].as<std::string>();
    int rows = config["rows"].as<int>();
    int cols = config["cols"].as<int>();
    this->params.use_history = config["use_history"].as<bool>();
    this->params.dt = config["dt"].as<double>();
    this->params.decimation = config["decimation"].as<int>();
    this->params.num_observations = config["num_observations"].as<int>();
    this->params.observations = ReadVectorFromYaml<std::string>(config["observations"]);
    this->params.clip_obs = config["clip_obs"].as<double>();
    if(config["clip_actions_lower"].IsNull() && config["clip_actions_upper"].IsNull())
    {
        this->params.clip_actions_upper = torch::tensor({}).view({1, -1});
        this->params.clip_actions_lower = torch::tensor({}).view({1, -1});
    }
    else
    {
        this->params.clip_actions_upper = torch::tensor(ReadVectorFromYaml<double>(config["clip_actions_upper"], this->params.framework, rows, cols)).view({1, -1});
        this->params.clip_actions_lower = torch::tensor(ReadVectorFromYaml<double>(config["clip_actions_lower"], this->params.framework, rows, cols)).view({1, -1});
    }
    this->params.action_scale = config["action_scale"].as<double>();
    this->params.hip_scale_reduction = config["hip_scale_reduction"].as<double>();
    this->params.hip_scale_reduction_indices = ReadVectorFromYaml<int>(config["hip_scale_reduction_indices"]);
    this->params.num_of_dofs = config["num_of_dofs"].as<int>();
    this->params.lin_vel_scale = config["lin_vel_scale"].as<double>();
    this->params.ang_vel_scale = config["ang_vel_scale"].as<double>();
    this->params.dof_pos_scale = config["dof_pos_scale"].as<double>();
    this->params.dof_vel_scale = config["dof_vel_scale"].as<double>();
    // this->params.commands_scale = torch::tensor(ReadVectorFromYaml<double>(config["commands_scale"])).view({1, -1});
    this->params.commands_scale = torch::tensor({this->params.lin_vel_scale, this->params.lin_vel_scale, this->params.ang_vel_scale});
    this->params.rl_kp = torch::tensor(ReadVectorFromYaml<double>(config["rl_kp"], this->params.framework, rows, cols)).view({1, -1});
    this->params.rl_kd = torch::tensor(ReadVectorFromYaml<double>(config["rl_kd"], this->params.framework, rows, cols)).view({1, -1});
    this->params.fixed_kp = torch::tensor(ReadVectorFromYaml<double>(config["fixed_kp"], this->params.framework, rows, cols)).view({1, -1});
    this->params.fixed_kd = torch::tensor(ReadVectorFromYaml<double>(config["fixed_kd"], this->params.framework, rows, cols)).view({1, -1});
    this->params.torque_limits = torch::tensor(ReadVectorFromYaml<double>(config["torque_limits"], this->params.framework, rows, cols)).view({1, -1});
    this->params.default_dof_pos = torch::tensor(ReadVectorFromYaml<double>(config["default_dof_pos"], this->params.framework, rows, cols)).view({1, -1});
    this->params.joint_controller_names = ReadVectorFromYaml<std::string>(config["joint_controller_names"], this->params.framework, rows, cols);
}

void RL::CSVInit(std::string robot_name)
{
    csv_filename = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + robot_name + "/motor";

    // Uncomment these lines if need timestamp for file name
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // std::stringstream ss;
    // ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    // std::string timestamp = ss.str();
    // csv_filename += "_" + timestamp;

    csv_filename += ".csv";
    std::ofstream file(csv_filename.c_str());

    for(int i = 0; i < 12; ++i) {file << "tau_cal_" << i << ",";}
    for(int i = 0; i < 12; ++i) {file << "tau_est_" << i << ",";}
    for(int i = 0; i < 12; ++i) {file << "joint_pos_" << i << ",";}
    for(int i = 0; i < 12; ++i) {file << "joint_pos_target_" << i << ",";}
    for(int i = 0; i < 12; ++i) {file << "joint_vel_" << i << ",";}

    file << std::endl;

    file.close();
}

void RL::CSVLogger(torch::Tensor torque, torch::Tensor tau_est, torch::Tensor joint_pos, torch::Tensor joint_pos_target, torch::Tensor joint_vel)
{
    std::ofstream file(csv_filename.c_str(), std::ios_base::app);

    for(int i = 0; i < 12; ++i) {file << torque[0][i].item<double>() << ",";}
    for(int i = 0; i < 12; ++i) {file << tau_est[0][i].item<double>() << ",";}
    for(int i = 0; i < 12; ++i) {file << joint_pos[0][i].item<double>() << ",";}
    for(int i = 0; i < 12; ++i) {file << joint_pos_target[0][i].item<double>() << ",";}
    for(int i = 0; i < 12; ++i) {file << joint_vel[0][i].item<double>() << ",";}

    file << std::endl;

    file.close();
}