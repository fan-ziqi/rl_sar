#include "rl.hpp"

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
