#include "pmtg_controller.hpp"
#include <stdexcept>
#include <iostream>
#include <numeric>

// 辅助函数：将vector转换为字符串，方便打印日志
template<typename T>
std::string vec_to_string(const std::vector<T>& vec) {
    if (vec.empty()) return "[]";
    std::string out = "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        out += std::to_string(vec[i]);
        if (i < vec.size() - 1) out += ", ";
    }
    out += "]";
    return out;
}

PMTGController::PMTGController(const YamlParams& params)
{
    // ---- 1. 从YAML配置文件中加载所有步态参数 ----
    gait_type = params.Get<std::string>("gait_type", "trot");
    base_frequency = params.Get<float>("base_frequency",1.8f);
    max_clearance = params.Get<float>("max_clearance", 0.20f);
    body_height = params.Get<float>("body_height", 0.30f);
    duty_factor = params.Get<float>("duty_factor", 0.60f);
    max_horizontal_offset = params.Get<float>("max_horizontal_offset", 0.25f);
    
    // 从YAML加载并生成Z轴轨迹函数 (抬腿和落腿)
    auto z_updown_func_names = params.Get<std::vector<std::string>>("z_updown_height_func", {"sin", "sin"});
    if(z_updown_func_names.size() != 2) {
        throw std::runtime_error("z_updown_height_func must contain two function names.");
    }
    f_up = gen_func(z_updown_func_names[0]);
    f_down = gen_func(z_updown_func_names[1]);

    // ---- 2. 根据步态类型初始化腿部的初始相位 ----
    // 相位决定了四条腿的运动顺序
    // 顺序: FL, FR, RL, RR
    if (gait_type == "trot") {
        initial_phase = {0.f, M_PI, M_PI, 0.f}; // 对角腿相位相同
    } else if (gait_type == "walk") {
        initial_phase = {0.f, M_PI / 2.f, M_PI, M_PI * 1.5f};
    } else if (gait_type == "bound") {
        initial_phase = {0.f, 0.f, M_PI, M_PI}; // 前后腿相位相同
    } else {
        throw std::invalid_argument("Unsupported gait type: " + gait_type);
    }
    
    // ---- 3. 调用reset()来初始化所有内部状态变量 ----
    reset();

    // ---- 4. 打印加载的参数用于调试 ----
    std::cout << "[PMTGController] Initialized successfully." << std::endl;
    std::cout << "  - Gait Type: " << gait_type << std::endl;
    std::cout << "  - Base Frequency: " << base_frequency << std::endl;
    std::cout << "  - Initial Phase (FL,FR,RL,RR): " << vec_to_string(initial_phase) << std::endl;
}

void PMTGController::reset()
{
    // 关键：为了与训练时的随机化保持一致，交换左右腿的初始相位。
    // 这使得机器人每次重置后可能以不同的腿先起步，增加鲁棒性。
    // 交换 FL 和 FR 的相位
    std::swap(initial_phase[0], initial_phase[1]);
    // 交换 RL 和 RR 的相位
    std::swap(initial_phase[2], initial_phase[3]);

    // 重置内部时钟
    time_since_reset = 0.0f;
    // 将当前相位恢复为初始相位
    phi = initial_phase;
    // 清空上一时刻的动作历史
    delta_phi.assign(4, 0.0f);
    // 重新计算三角函数值
    cos_phi.resize(4);
    sin_phi.resize(4);
    is_swing.assign(4, false);
    for(int i=0; i<4; ++i) {
        cos_phi[i] = std::cos(phi[i]);
        sin_phi[i] = std::sin(phi[i]);
    }
}

std::vector<float> PMTGController::get_phase_info() const
{
    // 按照模型要求的顺序拼接13维观测向量
    std::vector<float> phase_info;
    phase_info.reserve(13);
    phase_info.insert(phase_info.end(), delta_phi.begin(), delta_phi.end()); // 上一时刻的相位增量动作 (4维)
    phase_info.insert(phase_info.end(), cos_phi.begin(), cos_phi.end());     // 当前相位的余弦 (4维)
    phase_info.insert(phase_info.end(), sin_phi.begin(), sin_phi.end());     // 当前相位的正弦 (4维)
    phase_info.push_back(base_frequency);                                    // 步频 (1维)
    return phase_info;
}

std::function<float(float)> PMTGController::gen_func(const std::string& func_name) {
    if (func_name == "cubic_up") {
        return [](float x) { return -16 * std::pow(x, 3) + 12 * std::pow(x, 2); };
    } else if (func_name == "cubic_down") {
        return [](float x) { return 16 * std::pow(x, 3) - 36 * std::pow(x, 2) + 24 * x - 4; };
    } else if (func_name == "linear_down") {
        return [](float x) { return 2.0f - 2.0f * x; };
    } else if (func_name == "sin") {
        return [](float x) { return std::sin(M_PI * x); };
    } else {
        throw std::invalid_argument("Unsupported PMTG z height function: " + func_name);
    }
}

std::vector<float> PMTGController::step(const std::vector<float>& delta_phi_action, const std::vector<float>& commands, float dt)
{
    // 核心执行函数，分为四步：

    // 1. 更新内部相位状态
    update_phase(delta_phi_action, dt);

    // 2. 计算足端在各自髋关节坐标系下的目标位置 (初始时xyz为0)
    //    这一步主要计算水平方向(X/Y)的位移
    auto foot_positions = compute_foot_positions_in_hip_frame(commands);
    
    // 3. 计算并叠加足端的Z轴高度轨迹
    //    摆动腿根据相位抬高，支撑腿保持在地面高度
    for (int i = 0; i < 4; ++i) {
        if (is_swing[i]) {
            // 将当前摆动相的相位归一化到 [0, 1) 区间
            float swing_phi_unclamped = (phi[i] / (2 * M_PI) - duty_factor) / (1.0f - duty_factor);
            float swing_phi = std::max(0.0f, std::min(1.0f, swing_phi_unclamped));

            // 根据归一化相位，使用预设的函数计算抬腿高度
            float factor = (swing_phi < 0.5f) ? f_up(swing_phi) : f_down(swing_phi);
            foot_positions[i][2] = factor * max_clearance - body_height;
        } else {
            // 支撑腿的目标Z轴位置就是身体高度
            foot_positions[i][2] = -body_height;
        }
    }

    // 4. 通过逆运动学(IK)解算，将足端三维空间位置转换为三个关节的目标角度
    return foot_position_in_hip_frame_to_joint_angle(foot_positions);
}

void PMTGController::update_phase(const std::vector<float>& delta_phi_action, float dt)
{
    // 累积内部时钟
    time_since_reset += dt;
    // 存储当前传入的动作，这个值将在下一个控制周期被 get_phase_info() 作为观测返回给模型
    delta_phi = delta_phi_action; 

    // 对每条腿进行相位更新
    for (int i = 0; i < 4; ++i) {
        // 核心相位更新公式：
        // 新相位 = (初始相位 + 基础频率演化的相位 + 模型输出的相位增量) mod 2PI
        phi[i] = std::fmod(initial_phase[i] + 2 * M_PI * base_frequency * time_since_reset + delta_phi[i], 2 * M_PI);
        if (phi[i] < 0) {
            phi[i] += 2 * M_PI; // 确保相位在 [0, 2PI)
        }
        // 更新相位的三角函数值，供观测和计算使用
        cos_phi[i] = std::cos(phi[i]);
        sin_phi[i] = std::sin(phi[i]);
        // 根据相位和占空比判断腿是否处于摆动相
        is_swing[i] = (phi[i] / (2 * M_PI)) >= duty_factor;
    }
}

std::vector<std::vector<float>> PMTGController::compute_foot_positions_in_hip_frame(const std::vector<float>& commands) const
{
    // 初始化足端目标位置为(0,0,0) - 对应Python代码中 residual_xyz 为零的情况
    std::vector<std::vector<float>> foot_positions(4, std::vector<float>(3, 0.0f));
    
    // 只对摆动腿计算水平位移
    for(int i = 0; i < 4; ++i) {
        if(is_swing[i]) {
            // 将当前摆动相的相位归一化到 [0, 1) 区间
            float swing_phi_unclamped = (phi[i] / (2 * M_PI) - duty_factor) / (1.0f - duty_factor);
            float swing_phi = std::max(0.0f, std::min(1.0f, swing_phi_unclamped));

            // 根据归一化相位计算足端在X轴的摆动轨迹
            // 这是一个固定的正弦轨迹，模拟迈步动作，与指令速度无关
            foot_positions[i][0] = -max_horizontal_offset * std::sin(swing_phi * 2 * M_PI);
            // TODO: 如果需要支持平移(vy)和旋转(vyaw)，可在这里添加对Y轴和旋转的计算
        }
    }
    return foot_positions;
}

std::vector<float> PMTGController::foot_position_in_hip_frame_to_joint_angle(const std::vector<std::vector<float>>& foot_positions) const
{
    std::vector<float> joint_angles(12);
    const float l_up = UPPER_LEG_LENGTH;
    const float l_low = LOWER_LEG_LENGTH;

    for (int i = 0; i < 4; ++i) {
        // ---- 单腿逆运动学解析解 ----
        // 详细推导可参考相关机器人学教材

        // 获取当前腿的参数
        const float l_hip = HIP_Y_OFFSET * l_hip_sign[i]; // 髋关节Y轴偏移
        const float x = foot_positions[i][0];
        const float y = foot_positions[i][1];
        const float z = foot_positions[i][2];

        // 1. 计算膝关节(calf)角度 (theta_knee)
        const float leg_sqr = x*x + y*y + z*z;
        float theta_knee_input = (leg_sqr - l_hip*l_hip - l_low*l_low - l_up*l_up) / (2 * l_low * l_up);
        theta_knee_input = std::max(-1.0f, std::min(1.0f, theta_knee_input)); // Clamp to valid arccos range
        const float theta_knee = -std::acos(theta_knee_input);

        // 2. 计算大腿(thigh)角度 (theta_hip)
        const float l = std::sqrt(l_up*l_up + l_low*l_low + 2 * l_up * l_low * std::cos(theta_knee));
        float theta_hip_input = -x / l;
        theta_hip_input = std::max(-1.0f, std::min(1.0f, theta_hip_input)); // Clamp to valid arcsin range
        const float theta_hip = std::asin(theta_hip_input) - theta_knee / 2.0f;
        
        // 3. 计算髋关节(abduction)角度 (theta_ab)
        const float c1 = l_hip * y - l * std::cos(theta_hip + theta_knee / 2.0f) * z;
        const float s1 = l * std::cos(theta_hip + theta_knee / 2.0f) * y + l_hip * z;
        const float theta_ab = std::atan2(s1, c1);
        
        // 按照每条腿 [Hip, Thigh, Calf] 的顺序填充最终的12维向量
        // 腿部顺序为 FL, FR, RL, RR
        joint_angles[i*3 + 0] = theta_ab;
        joint_angles[i*3 + 1] = theta_hip;
        joint_angles[i*3 + 2] = theta_knee;
    }
    
    return joint_angles;
}
