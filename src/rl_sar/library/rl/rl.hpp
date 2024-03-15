#ifndef RL_HPP
#define RL_HPP

#include <torch/script.h>
#include <iostream>
#include <string>

struct ModelParams {
    int num_observations;
    float damping;
    float stiffness;
    float action_scale;
    float hip_scale_reduction;
    float num_of_dofs;
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float clip_obs;
    float clip_actions;
    torch::Tensor torque_limits;
    torch::Tensor d_gains;
    torch::Tensor p_gains;
    torch::Tensor commands_scale;
    torch::Tensor default_dof_pos;
};

struct Observations {
    torch::Tensor lin_vel;           
    torch::Tensor ang_vel;      
    torch::Tensor gravity_vec;      
    torch::Tensor commands;        
    torch::Tensor base_quat;   
    torch::Tensor dof_pos;           
    torch::Tensor dof_vel;           
    torch::Tensor actions;
};

class RL {
public:
    RL(){};
    ModelParams params;
    Observations obs;

    virtual torch::Tensor forward() = 0;
    virtual torch::Tensor compute_observation() = 0;
    torch::Tensor compute_torques(torch::Tensor actions);
    torch::Tensor compute_pos(torch::Tensor actions);
    torch::Tensor quat_rotate_inverse(torch::Tensor q, torch::Tensor v);
    void init_observations();

protected:
    // rl module
    torch::jit::script::Module actor;
    // observation buffer
    torch::Tensor lin_vel;           
    torch::Tensor ang_vel;      
    torch::Tensor gravity_vec;      
    torch::Tensor commands;        
    torch::Tensor base_quat;   
    torch::Tensor dof_pos;           
    torch::Tensor dof_vel;           
    torch::Tensor actions;

    torch::Tensor torques = torch::tensor({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    torch::Tensor target_dof_pos;
};

#endif // RL_HPP