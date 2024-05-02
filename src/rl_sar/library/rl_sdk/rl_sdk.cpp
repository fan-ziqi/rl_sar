#include "rl_sdk.hpp"

torch::Tensor ReadTensorFromYaml(const YAML::Node& node)
{
    std::vector<float> values;
    for(const auto& val : node)
    {
        values.push_back(val.as<float>());
    }
    return torch::tensor(values).view({1, -1});
}

void RL::ReadYaml(std::string robot_name)
{
	YAML::Node config;
	try
	{
		config = YAML::LoadFile(CONFIG_PATH)[robot_name];
	} catch(YAML::BadFile &e)
	{

		std::cout << "The file '" << CONFIG_PATH << "' does not exist" << std::endl;
		return;
	}

    this->params.model_name = config["model_name"].as<std::string>();
    this->params.num_observations = config["num_observations"].as<int>();
    this->params.clip_obs = config["clip_obs"].as<float>();
    this->params.clip_actions = config["clip_actions"].as<float>();
    this->params.action_scale = config["action_scale"].as<float>();
    this->params.hip_scale_reduction = config["hip_scale_reduction"].as<float>();
    this->params.num_of_dofs = config["num_of_dofs"].as<int>();
    this->params.lin_vel_scale = config["lin_vel_scale"].as<float>();
    this->params.ang_vel_scale = config["ang_vel_scale"].as<float>();
    this->params.dof_pos_scale = config["dof_pos_scale"].as<float>();
    this->params.dof_vel_scale =config["dof_vel_scale"].as<float>();
    this->params.commands_scale = torch::tensor({this->params.lin_vel_scale, this->params.lin_vel_scale, this->params.ang_vel_scale * 2});
    // this->params.damping = config["damping"].as<float>();
    // this->params.stiffness = config["stiffness"].as<float>();
    // this->params.d_gains = torch::ones(12) * this->params.damping;
    // this->params.p_gains = torch::ones(12) * this->params.stiffness;
    this->params.p_gains = ReadTensorFromYaml(config["p_gains"]);
    this->params.d_gains = ReadTensorFromYaml(config["d_gains"]);
    this->params.torque_limits = ReadTensorFromYaml(config["torque_limits"]);
    this->params.default_dof_pos = ReadTensorFromYaml(config["default_dof_pos"]);
    const YAML::Node& joint_names_node = config["joint_names"];
    for(const auto& name : joint_names_node)
    {
        this->params.joint_names.push_back(name.as<std::string>());
    }
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
    std::ofstream file(csv_filename.c_str()); // 创建新文件

    for(int i = 0; i < 12; ++i) {file << "torque_" << i << ",";}
    for(int i = 0; i < 12; ++i) {file << "tau_est_" << i << ",";}
    for(int i = 0; i < 12; ++i) {file << "joint_pos_" << i << ",";}
    for(int i = 0; i < 12; ++i) {file << "joint_pos_target_" << i << ",";}
    for(int i = 0; i < 12; ++i) {file << "joint_vel_" << i << ",";}

    file << std::endl;

    file.close(); // 关闭文件
}

void RL::CSVLogger(torch::Tensor torque, torch::Tensor tau_est, torch::Tensor joint_pos, torch::Tensor joint_pos_target, torch::Tensor joint_vel)
{
    std::ofstream file(csv_filename.c_str(), std::ios_base::app);   // 打开文件以追加模式写入数据

    // 写入数据
    for(int i = 0; i < 12; ++i) {file << torque[0][i].item<double>() << ",";}
    for(int i = 0; i < 12; ++i) {file << tau_est[0][i].item<double>() << ",";}
    for(int i = 0; i < 12; ++i) {file << joint_pos[0][i].item<double>() << ",";}
    for(int i = 0; i < 12; ++i) {file << joint_pos_target[0][i].item<double>() << ",";}
    for(int i = 0; i < 12; ++i) {file << joint_vel[0][i].item<double>() << ",";}

    file << std::endl;

    file.close(); // 关闭文件
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

void RL::InitObservations()
{
    this->obs.lin_vel = torch::tensor({{0.0, 0.0, 0.0}});
    this->obs.ang_vel = torch::tensor({{0.0, 0.0, 0.0}});
    this->obs.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    this->obs.commands = torch::tensor({{0.0, 0.0, 0.0}});
    this->obs.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    this->obs.dof_pos = this->params.default_dof_pos;
    this->obs.dof_vel = torch::tensor({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    this->obs.actions = torch::tensor({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
}

void RL::InitOutputs()
{
    output_torques = torch::tensor({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    output_dof_pos = params.default_dof_pos;
}

torch::Tensor RL::ComputeTorques(torch::Tensor actions)
{
    torch::Tensor actions_scaled = actions * this->params.action_scale;
    torch::Tensor output_torques = this->params.p_gains * (actions_scaled + this->params.default_dof_pos - this->obs.dof_pos) - this->params.d_gains * this->obs.dof_vel;
    torch::Tensor clamped = torch::clamp(output_torques, -(this->params.torque_limits), this->params.torque_limits);
    return clamped;
}

torch::Tensor RL::ComputePosition(torch::Tensor actions)
{
    torch::Tensor actions_scaled = actions * this->params.action_scale;
    return actions_scaled + this->params.default_dof_pos;
}

/* You may need to override this ComputeObservation() function
torch::Tensor RL::ComputeObservation()
{
    torch::Tensor obs = torch::cat({(this->QuatRotateInverse(this->base_quat, this->lin_vel)) * this->params.lin_vel_scale,
                                    (this->QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel)) * this->params.ang_vel_scale,
                                    this->QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec),
                                    this->obs.commands * this->params.commands_scale,
                                    (this->obs.dof_pos - this->params.default_dof_pos) * this->params.dof_pos_scale,
                                    this->obs.dof_vel * this->params.dof_vel_scale,
                                    this->obs.actions},
                                   1);

    obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);

    printf("observation size: %d, %d\n", obs.sizes()[0], obs.sizes()[1]);

    return obs;
}
*/

/* You may need to override this Forward() function
torch::Tensor RL::Forward()
{
    torch::Tensor obs = this->ComputeObservation();

    torch::Tensor actor_input = torch::cat({obs}, 1);

    torch::Tensor action = this->actor.forward({actor_input}).toTensor();

    this->obs.actions = action;
    torch::Tensor clamped = torch::clamp(action, -this->params.clip_actions, this->params.clip_actions);

    return clamped;
}
*/
