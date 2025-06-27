/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef G1_FSM_HPP
#define G1_FSM_HPP

#include "fsm_core.hpp"
#include "rl_sdk.hpp"

namespace g1_fsm
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
            else if (rl.control.control_state == STATE::STATE_RL_SKILL_1)
            {
                return "RLFSMStateRL_Dance";
            }
            else if (rl.control.control_state == STATE::STATE_RL_SKILL_2)
            {
                return "RLFSMStateRL_KungFu";
            }
            else if (rl.control.control_state == STATE::STATE_RL_SKILL_3)
            {
                return "RLFSMStateRL_Kick";
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
        else if (rl.control.control_state == STATE::STATE_RL_SKILL_1)
        {
            return "RLFSMStateRL_Dance";
        }
        else if (rl.control.control_state == STATE::STATE_RL_SKILL_2)
        {
            return "RLFSMStateRL_KungFu";
        }
        else if (rl.control.control_state == STATE::STATE_RL_SKILL_3)
        {
            return "RLFSMStateRL_Kick";
        }
        else if (rl.control.control_state == STATE::STATE_WAITING)
        {
            return "RLFSMStateWaiting";
        }
        return state_name_;
    }
};

class RLFSMStateRL_Dance : public RLFSMState
{
public:
    RLFSMStateRL_Dance(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_Dance") {}

    void Enter() override
    {
        // read params from yaml
        rl.config_name = "robomimic_dance";
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

        rl.episode_length_buf = 0;
        rl.motion_length = 18.0;

        // pos init
    }

    void Run() override
    {
        float motion_time = rl.episode_length_buf * rl.params.dt * rl.params.decimation;
        motion_time = fmin(motion_time, rl.motion_length);
        float running_progress = motion_time / rl.motion_length * 100.0f;
        std::cout << "\r" << std::flush << LOGGER::INFO << "Running progress "<< std::fixed << std::setprecision(2) << running_progress << "%" << std::flush;

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

        if (motion_time / rl.motion_length == 1)
        {
            rl.control.SetControlState(STATE::STATE_POS_GETUP);
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

class RLFSMStateRL_KungFu : public RLFSMState
{
public:
    RLFSMStateRL_KungFu(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_KungFu") {}

    void Enter() override
    {
        // read params from yaml
        rl.config_name = "robomimic_kungfu";
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

        rl.episode_length_buf = 0;
        rl.motion_length = 17.433;

        // pos init
    }

    void Run() override
    {
        float motion_time = rl.episode_length_buf * rl.params.dt * rl.params.decimation;
        motion_time = fmin(motion_time, rl.motion_length);
        float running_progress = motion_time / rl.motion_length * 100.0f;
        std::cout << "\r" << std::flush << LOGGER::INFO << "Running progress "<< std::fixed << std::setprecision(2) << running_progress << "%" << std::flush;

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

        if (motion_time / rl.motion_length == 1)
        {
            rl.control.SetControlState(STATE::STATE_POS_GETUP);
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

class RLFSMStateRL_Kick : public RLFSMState
{
public:
    RLFSMStateRL_Kick(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_Kick") {}

    void Enter() override
    {
        // read params from yaml
        rl.config_name = "robomimic_kick";
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

        rl.episode_length_buf = 0;
        rl.motion_length = 3.633;

        // pos init
    }

    void Run() override
    {
        float motion_time = rl.episode_length_buf * rl.params.dt * rl.params.decimation;
        motion_time = fmin(motion_time, rl.motion_length);
        float running_progress = motion_time / rl.motion_length * 100.0f;
        std::cout << "\r" << std::flush << LOGGER::INFO << "Running progress "<< std::fixed << std::setprecision(2) << running_progress << "%" << std::flush;

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

        if (motion_time / rl.motion_length == 1)
        {
            rl.control.SetControlState(STATE::STATE_POS_GETUP);
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

} // namespace g1_fsm

class G1FSMFactory : public FSMFactory
{
public:
    G1FSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStateWaiting")
            return std::make_shared<g1_fsm::RLFSMStateWaiting>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<g1_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<g1_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRL_Locomotion")
            return std::make_shared<g1_fsm::RLFSMStateRL_Locomotion>(rl);
        else if (state_name == "RLFSMStateRL_Dance")
            return std::make_shared<g1_fsm::RLFSMStateRL_Dance>(rl);
        else if (state_name == "RLFSMStateRL_KungFu")
            return std::make_shared<g1_fsm::RLFSMStateRL_KungFu>(rl);
        else if (state_name == "RLFSMStateRL_Kick")
            return std::make_shared<g1_fsm::RLFSMStateRL_Kick>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "g1"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStateWaiting",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRL_Locomotion",
            "RLFSMStateRL_Dance",
            "RLFSMStateRL_KungFu",
            "RLFSMStateRL_Kick"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(G1FSMFactory, "RLFSMStateWaiting")

#endif // G1_FSM_HPP
