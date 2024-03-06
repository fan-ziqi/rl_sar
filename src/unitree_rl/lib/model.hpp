#ifndef MODEL_HPP
#define MODEL_HPP

#include <torch/script.h>
#include <iostream>
#include <string>

struct ModelParams {
    int num_observations;
    float damping;
    float stiffness;
    float action_scale;
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

class Model {
public:
    Model(){};
    ModelParams params;
    Observations obs;

    virtual torch::Tensor forward();
    virtual torch::Tensor compute_observation();

    torch::Tensor compute_torques(torch::Tensor actions);
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
};

#endif // MODEL_HPP