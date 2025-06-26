/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef B2W_FSM_HPP
#define B2W_FSM_HPP

#include "fsm_core.hpp"
#include "rl_sdk.hpp"

namespace b2w_fsm
{

class RLFSMStateWaiting : public RLFSMState
{
public:
    RLFSMStateWaiting(RL *rl) : RLFSMState(*rl, "RLFSMStateWaiting") {}

    void Enter() override
    {
        rl.running_percent = 0.0f;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.control_state == STATE::STATE_POS_GETUP)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    void Enter() override
    {
        rl.running_percent = 0.0f;
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        if (rl.running_percent < 1.0f)
        {
            rl.running_percent += 1.0f / 500.0f;
            rl.running_percent = std::min(rl.running_percent, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                fsm_command->motor_command.q[i] = (1 - rl.running_percent) * rl.now_state.motor_state.q[i] + rl.running_percent * rl.params.default_dof_pos[0][i].item<double>();
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0;
            }
            std::cout << "\r" << std::flush << LOGGER::INFO << "Getting up " << std::fixed << std::setprecision(2) << rl.running_percent * 100.0f << "%" << std::flush;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.running_percent >= 1.0f)
        {
            if (rl.control.control_state == STATE::STATE_RL_LOCOMOTION)
            {
                return "RLFSMStateRL_Locomotion";
            }
            else if (rl.control.control_state == STATE::STATE_POS_GETDOWN)
            {
                return "RLFSMStateGetDown";
            }
            else if (rl.control.control_state == STATE::STATE_WAITING)
            {
                return "RLFSMStateWaiting";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    void Enter() override
    {
        rl.running_percent = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        if (rl.running_percent < 1.0f)
        {
            rl.running_percent += 1.0f / 500.0f;
            rl.running_percent = std::min(rl.running_percent, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                fsm_command->motor_command.q[i] = (1 - rl.running_percent) * rl.now_state.motor_state.q[i] + rl.running_percent * rl.start_state.motor_state.q[i];
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0;
            }
            std::cout << "\r" << std::flush << LOGGER::INFO << "Getting down "<< std::fixed << std::setprecision(2) << rl.running_percent * 100.0f << "%" << std::flush;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.running_percent >= 1.0f)
        {
            return "RLFSMStateWaiting";
        }
        else if (rl.control.control_state == STATE::STATE_POS_GETUP)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateRL_Locomotion : public RLFSMState
{
public:
    RLFSMStateRL_Locomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_Locomotion") {}

    void Enter() override
    {
        // read params from yaml
        rl.config_name = rl.default_rl_config;
        std::string robot_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_path);
            rl.rl_init_done = true;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.control.control_state = STATE::STATE_POS_GETUP;
        }

        // pos init
    }

    void Run() override
    {
        std::cout << "\r" << std::flush << LOGGER::INFO << "RL Controller x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;

        torch::Tensor _output_dof_pos, _output_dof_vel;
        if (rl.output_dof_pos_queue.try_pop(_output_dof_pos) && rl.output_dof_vel_queue.try_pop(_output_dof_vel))
        {
            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                if (_output_dof_pos.defined() && _output_dof_pos.numel() > 0)
                {
                    fsm_command->motor_command.q[i] = rl.output_dof_pos[0][i].item<double>();
                }
                if (_output_dof_vel.defined() && _output_dof_vel.numel() > 0)
                {
                    fsm_command->motor_command.dq[i] = rl.output_dof_vel[0][i].item<double>();
                }
                fsm_command->motor_command.kp[i] = rl.params.rl_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.rl_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0;
            }
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.control_state == STATE::STATE_POS_GETDOWN)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.control_state == STATE::STATE_POS_GETUP)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.control_state == STATE::STATE_RL_LOCOMOTION)
        {
            return "RLFSMStateRL_Locomotion";
        }
        else if (rl.control.control_state == STATE::STATE_WAITING)
        {
            return "RLFSMStateWaiting";
        }
        return state_name_;
    }
};

} // namespace b2w_fsm

class B2WFSMFactory : public FSMFactory
{
public:
    B2WFSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStateWaiting")
            return std::make_shared<b2w_fsm::RLFSMStateWaiting>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<b2w_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<b2w_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRL_Locomotion")
            return std::make_shared<b2w_fsm::RLFSMStateRL_Locomotion>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "b2w"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStateWaiting",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRL_Locomotion"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(B2WFSMFactory, "RLFSMStateWaiting")

#endif // B2W_FSM_HPP
