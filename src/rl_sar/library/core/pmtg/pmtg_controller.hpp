#ifndef PMTG_CONTROLLER_HPP
#define PMTG_CONTROLLER_HPP

#include <vector>
#include <string>
#include <cmath>
#include <functional>
#include <stdexcept>
#include "../rl_sdk/rl_sdk.hpp" // For YamlParams

/**
 * @class PMTGController
 * @brief Phase-Modulated Trajectory Generator (相位调制轨迹生成器)
 * 
 * 该类将Python训练框架中的PMTrajectoryGenerator移植到C++。
 * 它根据给定的步态参数和来自神经网络的动作指令（相位增量），
 * 生成一个基础的四足机器人步态轨迹（目标关节角度）。
 * 核心功能包括：
 * 1. 内部相位状态的维护与更新。
 * 2. 根据相位生成足端轨迹。
 * 3. 通过逆运动学（IK）解算得到目标关节角度。
 */
class PMTGController
{
public:
    /**
     * @brief 构造函数，从YAML配置中加载所有PMTG相关参数。
     * @param params 包含所有配置参数的YamlParams对象。
     */
    PMTGController(const YamlParams& params);

    /**
     * @brief 重置控制器内部状态，包括相位、时间和上一时刻的动作。
     *        通常在一个回合（episode）开始时调用。
     */
    void reset();

    /**
     * @brief 获取13维的相位观测信息，用于输入神经网络。
     * @return std::vector<float> 格式为 [delta_phi(4), cos_phi(4), sin_phi(4), frequency(1)]。
     *         这是一个只读操作，不会改变控制器内部状态。
     */
    std::vector<float> get_phase_info() const;

    /**
     * @brief 驱动控制器前进一个时间步，更新状态并计算输出。
     * @param delta_phi_action 模型输出的4维相位增量，用于调制相位。
     * @param commands 用户输入的3维速度指令 [vx, vy, vyaw]，用于计算足端水平位移。
     * @param dt 控制周期（例如 0.01s），用于驱动内部时钟。
     * @return 计算出的12维基础关节目标角度。
     */
    std::vector<float> step(const std::vector<float>& delta_phi_action, const std::vector<float>& commands, float dt);

private:
    // ---- 内部辅助函数 ----

    /**
     * @brief 根据模型动作和时间步长更新内部相位(phi)。
     */
    void update_phase(const std::vector<float>& delta_phi_action, float dt);
    
    /**
     * @brief 根据用户速度指令计算足端在各自髋关节坐标系下的目标位置（主要影响水平面）。
     */
    std::vector<std::vector<float>> compute_foot_positions_in_hip_frame(const std::vector<float>& commands) const;
    
    /**
     * @brief 逆运动学解算：将足端在髋关节坐标系下的目标位置转换为三个关节的角度。
     */
    std::vector<float> foot_position_in_hip_frame_to_joint_angle(const std::vector<std::vector<float>>& foot_positions) const;
    
    /**
     * @brief 根据函数名字符串生成对应的数学函数（用于Z轴轨迹）。
     */
    std::function<float(float)> gen_func(const std::string& func_name);

    // ---- 从YAML加载的参数 ----
    std::string gait_type;          ///< 步态类型 (e.g., "trot")
    float base_frequency;           ///< 基础步频 (Hz)
    float max_clearance;            ///< 摆动腿最大抬腿高度 (m)
    float body_height;              ///< 身体高度 (m), 用于计算足端Z轴基准
    float duty_factor;              ///< 占空比 (支撑相占整个周期的比例)
    float max_horizontal_offset;    ///< 摆动腿在前进方向上的最大水平偏移
    std::function<float(float)> f_up;   ///< 摆动腿抬起时的Z轴轨迹函数
    std::function<float(float)> f_down; ///< 摆动腿下落时的Z轴轨迹函数

    // ---- 机器人物理参数 (来自 go2 ament, 您可以后续调整或移至YAML) ----
    const float UPPER_LEG_LENGTH = 0.213;   ///< 大腿连杆长度 (m)
    const float LOWER_LEG_LENGTH = 0.213;   ///< 小腿连杆长度 (m)
    const float HIP_Y_OFFSET = 0.0935;      ///< 髋关节(abduction)的Y轴偏移量 (m)
    /**
     * @brief 髋关节在机器人身体坐标系下的偏移量。
     * @note 顺序为物理正确的 [FL, FR, RL, RR]。
     */
    const std::vector<std::vector<float>> HIP_OFFSETS_BASE_FRAME = {
        { 0.23,  0.051, 0.}, // FL
        { 0.23, -0.051, 0.}, // FR
        {-0.23,  0.051, 0.}, // RL
        {-0.23, -0.051, 0.}  // RR
    };

    /**
     * @brief 髋关节l_hip的符号，用于IK计算。
     * @note 顺序为物理正确的 [FL, FR, RL, RR]，定义为 左腿=+1, 右腿=-1。
     */
    const std::vector<float> l_hip_sign = {1.f, -1.f, 1.f, -1.f};

    // ---- 内部状态变量 ----
    float time_since_reset;                 ///< 从上次重置开始累计的时间 (s)
    std::vector<float> initial_phase;       ///< 各腿的初始相位
    std::vector<float> phi;                 ///< 各腿当前相位 (rad)
    std::vector<float> cos_phi;             ///< 各腿当前相位的余弦值
    std::vector<float> sin_phi;             ///< 各腿当前相位的正弦值
    std::vector<float> delta_phi;           ///< 上一步模型输出的相位增量 (用于构建下一次观测)
    std::vector<bool> is_swing;             ///< 标记各腿是否处于摆动相
    
};

#endif // PMTG_CONTROLLER_HPP
