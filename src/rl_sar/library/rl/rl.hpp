#ifndef RL_HPP
#define RL_HPP

#include <torch/script.h>
#include <iostream>
#include <string>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

struct ModelParams
{
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

    ModelParams params;
    Observations obs;

    virtual torch::Tensor Forward() = 0;
    virtual torch::Tensor ComputeObservation() = 0;
    torch::Tensor ComputeTorques(torch::Tensor actions);
    torch::Tensor ComputePosition(torch::Tensor actions);
    torch::Tensor QuatRotateInverse(torch::Tensor q, torch::Tensor v);
    void InitObservations();

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
    // output buffer
    torch::Tensor output_torques;
    torch::Tensor output_dof_pos;
};

#endif // RL_HPP