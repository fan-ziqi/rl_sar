#include "rl_sdk.hpp"

/* You may need to override this ComputeObservation() function
torch::Tensor RL::ComputeObservation()
{
    torch::Tensor obs = torch::cat({this->QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) * this->params.ang_vel_scale,
                                    this->QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec),
                                    this->obs.commands * this->params.commands_scale,
                                    (this->obs.dof_pos - this->params.default_dof_pos) * this->params.dof_pos_scale,
                                    this->obs.dof_vel * this->params.dof_vel_scale,
                                    this->obs.actions
                                    },1);
    torch::Tensor clamped_obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);
    return clamped_obs;
}
*/

/* You may need to override this Forward() function
torch::Tensor RL::Forward()
{
    torch::autograd::GradMode::set_enabled(false);

    torch::Tensor clamped_obs = this->ComputeObservation();

    torch::Tensor actions = this->model.forward({clamped_obs}).toTensor();

    torch::Tensor clamped_actions = torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);

    return clamped_actions;
}
*/

void RL::InitObservations()
{
    this->obs.lin_vel = torch::tensor({{0.0, 0.0, 0.0}});
    this->obs.ang_vel = torch::tensor({{0.0, 0.0, 0.0}});
    this->obs.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    this->obs.commands = torch::tensor({{0.0, 0.0, 0.0}});
    this->obs.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    this->obs.dof_pos = this->params.default_dof_pos;
    this->obs.dof_vel = torch::zeros({1, params.num_of_dofs});
    this->obs.actions = torch::zeros({1, params.num_of_dofs});
}

void RL::InitOutputs()
{
    this->output_torques = torch::zeros({1, params.num_of_dofs});
    this->output_dof_pos = params.default_dof_pos;
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

torch::Tensor RL::QuatRotateInverse(torch::Tensor q, torch::Tensor v)
{
    c10::IntArrayRef shape = q.sizes();
    torch::Tensor q_w = q.index({torch::indexing::Slice(), -1});
    torch::Tensor q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
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
    if(running_state == STATE_WAITING)
    {
        for(int i = 0; i < params.num_of_dofs; ++i)
        {
            command->motor_command.q[i] = state->motor_state.q[i];
        }
        if(control.control_state == STATE_POS_GETUP)
        {
            control.control_state = STATE_WAITING;
            getup_percent = 0.0;
            for(int i = 0; i < params.num_of_dofs; ++i)
            {
                now_state.motor_state.q[i] = state->motor_state.q[i];
                start_state.motor_state.q[i] = now_state.motor_state.q[i];
            }
            running_state = STATE_POS_GETUP;
        }
    }
    // stand up (position control)
    else if(running_state == STATE_POS_GETUP)
    {
        if(getup_percent < 1.0)
        {
            getup_percent += 1 / 1000.0;
            getup_percent = getup_percent > 1.0 ? 1.0 : getup_percent;
            for(int i = 0; i < params.num_of_dofs; ++i)
            {
                command->motor_command.q[i] = (1 - getup_percent) * now_state.motor_state.q[i] + getup_percent * params.default_dof_pos[0][i].item<double>();
                command->motor_command.dq[i] = 0;
                command->motor_command.kp[i] = params.fixed_kp[0][i].item<double>();
                command->motor_command.kd[i] = params.fixed_kd[0][i].item<double>();
                command->motor_command.tau[i] = 0;
            }
            std::cout << LOGGER::INFO << "Getting up " << std::fixed << std::setprecision(2) << getup_percent * 100.0 << "%\r";
        }
        if(control.control_state == STATE_RL_INIT)
        {
            std::cout << std::endl;
            control.control_state = STATE_WAITING;
            running_state = STATE_RL_INIT;
        }
        else if(control.control_state == STATE_POS_GETDOWN)
        {
            control.control_state = STATE_WAITING;
            getdown_percent = 0.0;
            for(int i = 0; i < params.num_of_dofs; ++i)
            {
                now_state.motor_state.q[i] = state->motor_state.q[i];
            }
            running_state = STATE_POS_GETDOWN;
        }
    }
    // init obs and start rl loop
    else if(running_state == STATE_RL_INIT)
    {
        if(getup_percent == 1)
        {
            running_state = STATE_RL_RUNNING;
            this->InitObservations();
            this->InitOutputs();
            this->InitControl();
        }
    }
    // rl loop
    else if(running_state == STATE_RL_RUNNING)
    {
        for(int i = 0; i < params.num_of_dofs; ++i)
        {
            command->motor_command.q[i] = output_dof_pos[0][i].item<double>();
            command->motor_command.dq[i] = 0;
            command->motor_command.kp[i] = params.rl_kp[0][i].item<double>();
            command->motor_command.kd[i] = params.rl_kd[0][i].item<double>();
            command->motor_command.tau[i] = 0;
        }
        if(control.control_state == STATE_POS_GETDOWN)
        {
            control.control_state = STATE_WAITING;
            getdown_percent = 0.0;
            for(int i = 0; i < params.num_of_dofs; ++i)
            {
                now_state.motor_state.q[i] = state->motor_state.q[i];
            }
            running_state = STATE_POS_GETDOWN;
        }
    }
    // get down (position control)
    else if(running_state == STATE_POS_GETDOWN)
    {
        if(getdown_percent < 1.0)
        {
            getdown_percent += 1 / 1000.0;
            getdown_percent = getdown_percent > 1.0 ? 1.0 : getdown_percent;
            for(int i = 0; i < params.num_of_dofs; ++i)
            {
                command->motor_command.q[i] = (1 - getdown_percent) * now_state.motor_state.q[i] + getdown_percent * start_state.motor_state.q[i];
                command->motor_command.dq[i] = 0;
                command->motor_command.kp[i] = params.fixed_kp[0][i].item<double>();
                command->motor_command.kd[i] = params.fixed_kd[0][i].item<double>();
                command->motor_command.tau[i] = 0;
            }
            std::cout << LOGGER::INFO << "Getting down " << std::fixed << std::setprecision(2) << getdown_percent * 100.0 << "%\r";
        }
        if(getdown_percent == 1)
        {
            std::cout << std::endl;
            running_state = STATE_WAITING;
            this->InitObservations();
            this->InitOutputs();
            this->InitControl();
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

            std::cout << LOGGER::ERROR << "Torque(" << index+1 << ")=" << value << " out of range(" << limit_lower << ", " << limit_upper << ")" << std::endl;
            std::cout << LOGGER::ERROR << "Switching to STATE_POS_GETDOWN"<< std::endl;
        }
        control.control_state = STATE_POS_GETDOWN;
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
    if(running_state == STATE_RL_RUNNING)
    {
        std::cout << LOGGER::INFO << "RL Controller x:" << control.x << " y:" << control.y << " yaw:" << control.yaw << "          \r";
    }

    if(kbhit())
    {
        int c = fgetc(stdin);
        switch(c)
        {
            case '0': control.control_state = STATE_POS_GETUP; break;
            case 'p': control.control_state = STATE_RL_INIT; break;
            case '1': control.control_state = STATE_POS_GETDOWN; break;
            case 'q': break;
            case 'w': control.x += 0.1; break;
            case 's': control.x -= 0.1; break;
            case 'a': control.yaw += 0.1; break;
            case 'd': control.yaw -= 0.1; break;
            case 'i': break;
            case 'k': break;
            case 'j': control.y += 0.1; break;
            case 'l': control.y -= 0.1; break;
            case ' ': control.x = 0; control.y = 0; control.yaw = 0; break;
            case 'r': control.control_state = STATE_RESET_SIMULATION; break;
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

void RL::ReadYaml(std::string robot_name)
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(CONFIG_PATH)[robot_name];
    } catch(YAML::BadFile &e)
    {

        std::cout << LOGGER::ERROR << "The file '" << CONFIG_PATH << "' does not exist" << std::endl;
        return;
    }

    this->params.model_name = config["model_name"].as<std::string>();
    this->params.num_observations = config["num_observations"].as<int>();
    this->params.clip_obs = config["clip_obs"].as<double>();
    this->params.clip_actions_upper = torch::tensor(ReadVectorFromYaml<double>(config["clip_actions_upper"])).view({1, -1});
    this->params.clip_actions_lower = torch::tensor(ReadVectorFromYaml<double>(config["clip_actions_lower"])).view({1, -1});
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
    this->params.rl_kp = torch::tensor(ReadVectorFromYaml<double>(config["rl_kp"])).view({1, -1});
    this->params.rl_kd = torch::tensor(ReadVectorFromYaml<double>(config["rl_kd"])).view({1, -1});
    this->params.fixed_kp = torch::tensor(ReadVectorFromYaml<double>(config["fixed_kp"])).view({1, -1});
    this->params.fixed_kd = torch::tensor(ReadVectorFromYaml<double>(config["fixed_kd"])).view({1, -1});
    this->params.torque_limits = torch::tensor(ReadVectorFromYaml<double>(config["torque_limits"])).view({1, -1});
    this->params.default_dof_pos = torch::tensor(ReadVectorFromYaml<double>(config["default_dof_pos"])).view({1, -1});
    this->params.joint_controller_names = ReadVectorFromYaml<std::string>(config["joint_controller_names"]);
}

void RL::CSVInit(std::string robot_name)
{
    csv_filename = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + robot_name + "/motor";

    // // Uncomment these lines if need timestamp for file name 
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // std::stringstream ss;
    // ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    // std::string timestamp = ss.str();
    // csv_filename += "_" + timestamp;

    csv_filename += ".csv";
    std::ofstream file(csv_filename.c_str());

    for(int i = 0; i < 12; ++i) {file << "torque_" << i << ",";}
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