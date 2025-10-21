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

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 8;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
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

    float percent_getup = 0.0f;

    void Enter() override
    {
        percent_getup = 0.0f;
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.default_dof_pos, 2.0f, "Getting up", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRL_Locomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
            {
                return "RLFSMStateRL_RoboMimicLoco";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
            {
                return "RLFSMStateRL_RoboMimicDance";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
            {
                return "RLFSMStateRL_RoboMimicKungFu";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
            {
                return "RLFSMStateRL_RoboMimicKick";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
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

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "unitree_rl_gym";
        std::string robot_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_path);
            rl.rl_init_done = true;
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done) return;

        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.default_dof_pos, 0.5f, "Policy transition", true)) return;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRL_Locomotion";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateRL_RoboMimicLoco";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        {
            return "RLFSMStateRL_RoboMimicDance";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
        {
            return "RLFSMStateRL_RoboMimicKungFu";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
        {
            return "RLFSMStateRL_RoboMimicKick";
        }
        return state_name_;
    }
};

class RLFSMStateRL_RoboMimicLoco : public RLFSMState
{
public:
    RLFSMStateRL_RoboMimicLoco(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_RoboMimicLoco") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robomimic/loco";
        std::string robot_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_path);
            rl.rl_init_done = true;
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done) return;

        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.default_dof_pos, 0.5f, "Policy transition", true)) return;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRL_Locomotion";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateRL_RoboMimicLoco";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        {
            return "RLFSMStateRL_RoboMimicDance";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
        {
            return "RLFSMStateRL_RoboMimicKungFu";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
        {
            return "RLFSMStateRL_RoboMimicKick";
        }
        return state_name_;
    }
};

class RLFSMStateRL_RoboMimicDance : public RLFSMState
{
public:
    RLFSMStateRL_RoboMimicDance(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_RoboMimicDance") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robomimic/dance";
        std::string robot_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_path);
            rl.rl_init_done = true;
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }

        rl.motion_length = 18.0;
    }

    void Run() override
    {
        if (!rl.rl_init_done) return;

        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.default_dof_pos, 0.5f, "Policy transition", true)) return;

        float motion_time = rl.episode_length_buf * rl.params.dt * rl.params.decimation;
        motion_time = fmin(motion_time, rl.motion_length);
        float running_progress = motion_time / rl.motion_length * 100.0f;
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] Running progress "<< std::fixed << std::setprecision(2) << running_progress << "%" << std::flush;

        RLControl();

        if (motion_time / rl.motion_length == 1)
        {
            rl.fsm.RequestStateChange("RLFSMStateRL_RoboMimicLoco");
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRL_Locomotion";
        }
        return state_name_;
    }
};

class RLFSMStateRL_RoboMimicKungFu : public RLFSMState
{
public:
    RLFSMStateRL_RoboMimicKungFu(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_RoboMimicKungFu") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robomimic/kungfu";
        std::string robot_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_path);
            rl.rl_init_done = true;
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }

        rl.motion_length = 17.433;
    }

    void Run() override
    {
        if (!rl.rl_init_done) return;

        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.default_dof_pos, 0.5f, "Policy transition", true)) return;

        float motion_time = rl.episode_length_buf * rl.params.dt * rl.params.decimation;
        motion_time = fmin(motion_time, rl.motion_length);
        float running_progress = motion_time / rl.motion_length * 100.0f;
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] Running progress "<< std::fixed << std::setprecision(2) << running_progress << "%" << std::flush;

        RLControl();

        if (motion_time / rl.motion_length == 1)
        {
            rl.fsm.RequestStateChange("RLFSMStateRL_RoboMimicLoco");
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRL_Locomotion";
        }
        return state_name_;
    }
};

class RLFSMStateRL_RoboMimicKick : public RLFSMState
{
public:
    RLFSMStateRL_RoboMimicKick(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_RoboMimicKick") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robomimic/kick";
        std::string robot_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_path);
            rl.rl_init_done = true;
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }

        rl.motion_length = 3.633;
    }

    void Run() override
    {
        if (!rl.rl_init_done) return;

        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.default_dof_pos, 0.5f, "Policy transition", true)) return;

        float motion_time = rl.episode_length_buf * rl.params.dt * rl.params.decimation;
        motion_time = fmin(motion_time, rl.motion_length);
        float running_progress = motion_time / rl.motion_length * 100.0f;
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] Running progress "<< std::fixed << std::setprecision(2) << running_progress << "%" << std::flush;

        RLControl();

        if (motion_time / rl.motion_length == 1)
        {
            rl.fsm.RequestStateChange("RLFSMStateRL_RoboMimicLoco");
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRL_Locomotion";
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
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<g1_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<g1_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<g1_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRL_Locomotion")
            return std::make_shared<g1_fsm::RLFSMStateRL_Locomotion>(rl);
        else if (state_name == "RLFSMStateRL_RoboMimicLoco")
            return std::make_shared<g1_fsm::RLFSMStateRL_RoboMimicLoco>(rl);
        else if (state_name == "RLFSMStateRL_RoboMimicDance")
            return std::make_shared<g1_fsm::RLFSMStateRL_RoboMimicDance>(rl);
        else if (state_name == "RLFSMStateRL_RoboMimicKungFu")
            return std::make_shared<g1_fsm::RLFSMStateRL_RoboMimicKungFu>(rl);
        else if (state_name == "RLFSMStateRL_RoboMimicKick")
            return std::make_shared<g1_fsm::RLFSMStateRL_RoboMimicKick>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "g1"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRL_Locomotion",
            "RLFSMStateRL_RoboMimicLoco",
            "RLFSMStateRL_RoboMimicDance",
            "RLFSMStateRL_RoboMimicKungFu",
            "RLFSMStateRL_RoboMimicKick"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(G1FSMFactory, "RLFSMStatePassive")

#endif // G1_FSM_HPP
