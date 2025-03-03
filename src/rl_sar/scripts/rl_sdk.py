# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import torch
import yaml
import os
import csv
from pynput import keyboard
from enum import Enum, auto

BASE_PATH = os.path.join(os.path.dirname(__file__), "../")

class LOGGER:
    INFO = "\033[0;37m[INFO]\033[0m "
    WARNING = "\033[0;33m[WARNING]\033[0m "
    ERROR = "\033[0;31m[ERROR]\033[0m "
    DEBUG = "\033[0;32m[DEBUG]\033[0m "

class RobotCommand:
    def __init__(self):
        self.motor_command = self.MotorCommand()

    class MotorCommand:
        def __init__(self):
            self.q = [0.0] * 32
            self.dq = [0.0] * 32
            self.tau = [0.0] * 32
            self.kp = [0.0] * 32
            self.kd = [0.0] * 32

class RobotState:
    def __init__(self):
        self.imu = self.IMU()
        self.motor_state = self.MotorState()

    class IMU:
        def __init__(self):
            self.quaternion = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
            self.gyroscope = [0.0, 0.0, 0.0]
            self.accelerometer = [0.0, 0.0, 0.0]

    class MotorState:
        def __init__(self):
            self.q = [0.0] * 32
            self.dq = [0.0] * 32
            self.ddq = [0.0] * 32
            self.tau_est = [0.0] * 32
            self.cur = [0.0] * 32

class STATE(Enum):
    STATE_WAITING = 0
    STATE_POS_GETUP = auto()
    STATE_RL_INIT = auto()
    STATE_RL_RUNNING = auto()
    STATE_POS_GETDOWN = auto()
    STATE_RESET_SIMULATION = auto()
    STATE_TOGGLE_SIMULATION = auto()

class Control:
    def __init__(self):
        self.control_state = STATE.STATE_WAITING
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

class ModelParams:
    def __init__(self):
        self.model_name = None
        self.framework = None
        self.dt = None
        self.decimation = None
        self.num_observations = None
        self.observations = None
        self.observations_history = None
        self.damping = None
        self.stiffness = None
        self.action_scale = None
        self.wheel_indices = None
        self.num_of_dofs = None
        self.lin_vel_scale = None
        self.ang_vel_scale = None
        self.dof_pos_scale = None
        self.dof_vel_scale = None
        self.clip_obs = None
        self.clip_actions_upper = None
        self.clip_actions_lower = None
        self.torque_limits = None
        self.rl_kd = None
        self.rl_kp = None
        self.fixed_kp = None
        self.fixed_kd = None
        self.commands_scale = None
        self.default_dof_pos = None
        self.joint_controller_names = None
        self.command_mapping = None
        self.state_mapping = None

class Observations:
    def __init__(self):
        self.lin_vel = None
        self.ang_vel = None
        self.gravity_vec = None
        self.commands = None
        self.base_quat = None
        self.dof_pos = None
        self.dof_vel = None
        self.actions = None

class RL:
    # Static variables
    start_state = RobotState()
    now_state = RobotState()
    getup_percent = 0.0
    getdown_percent = 0.0

    def __init__(self):
        ### public in cpp ###
        self.params = ModelParams()
        self.obs = Observations()

        self.robot_state = RobotState()
        self.robot_command = RobotCommand()

        # control
        self.control = Control()

        # others
        self.robot_name = ""
        self.running_state = STATE.STATE_RL_RUNNING  # default running_state set to STATE_RL_RUNNING
        self.simulation_running = False

        ### protected in cpp ###
        # rl module
        self.model = None
        self.walk_model = None
        self.stand_model = None

        # output buffer
        self.output_dof_tau = torch.zeros(1, 32)
        self.output_dof_pos = torch.zeros(1, 32)

    def ComputeObservation(self):
        obs_list = []
        for observation in self.params.observations:
            """
                The first argument of the QuatRotateInverse function is the quaternion representing the robot's orientation, and the second argument is in the world coordinate system. The function outputs the value of the second argument in the body coordinate system.
                In IsaacGym, the coordinate system for angular velocity is in the world coordinate system. During training, the angular velocity in the observation uses QuatRotateInverse to transform the coordinate system to the body coordinate system.
                In Gazebo, the coordinate system for angular velocity is also in the world coordinate system, so QuatRotateInverse is needed to transform the coordinate system to the body coordinate system.
                In some real robots like Unitree, if the coordinate system for the angular velocity is already in the body coordinate system, no transformation is necessary.
                Forgetting to perform the transformation or performing it multiple times may cause controller crashes when the rotation reaches 180 degrees.
            """
            if observation == "lin_vel":
                obs_list.append(self.obs.lin_vel * self.params.lin_vel_scale)
            elif observation == "ang_vel_body":
                obs_list.append(self.obs.ang_vel * self.params.ang_vel_scale)
            elif observation == "ang_vel_world":
                obs_list.append(self.QuatRotateInverse(self.obs.base_quat, self.obs.ang_vel, self.params.framework) * self.params.ang_vel_scale)
            elif observation == "gravity_vec":
                obs_list.append(self.QuatRotateInverse(self.obs.base_quat, self.obs.gravity_vec, self.params.framework))
            elif observation == "commands":
                obs_list.append(self.obs.commands * self.params.commands_scale)
            elif observation == "dof_pos":
                dof_pos_rel = self.obs.dof_pos - self.params.default_dof_pos
                for i in self.params.wheel_indices:
                    dof_pos_rel[0, i] = 0.0
                obs_list.append(dof_pos_rel * self.params.dof_pos_scale)
            elif observation == "dof_vel":
                obs_list.append(self.obs.dof_vel * self.params.dof_vel_scale)
            elif observation == "actions":
                obs_list.append(self.obs.actions)
        obs = torch.cat(obs_list, dim=-1)
        clamped_obs = torch.clamp(obs, -self.params.clip_obs, self.params.clip_obs)
        return clamped_obs

    def InitObservations(self):
        self.obs.lin_vel = torch.zeros(1, 3, dtype=torch.float)
        self.obs.ang_vel = torch.zeros(1, 3, dtype=torch.float)
        self.obs.gravity_vec = torch.tensor([[0.0, 0.0, -1.0]])
        self.obs.commands = torch.zeros(1, 3, dtype=torch.float)
        self.obs.base_quat = torch.zeros(1, 4, dtype=torch.float)
        self.obs.dof_pos = self.params.default_dof_pos
        self.obs.dof_vel = torch.zeros(1, self.params.num_of_dofs, dtype=torch.float)
        self.obs.actions = torch.zeros(1, self.params.num_of_dofs, dtype=torch.float)

    def InitOutputs(self):
        self.output_dof_tau = torch.zeros(1, self.params.num_of_dofs, dtype=torch.float)
        self.output_dof_pos = self.params.default_dof_pos

    def InitControl(self):
        self.control.control_state = STATE.STATE_WAITING
        self.control.x = 0.0
        self.control.y = 0.0
        self.control.yaw = 0.0

    def ComputeTorques(self, actions):
        actions_scaled = actions * self.params.action_scale
        output_dof_tau = self.params.rl_kp * (actions_scaled + self.params.default_dof_pos - self.obs.dof_pos) - self.params.rl_kd * self.obs.dof_vel
        return output_dof_tau

    def ComputePosition(self, actions):
        actions_scaled = actions * self.params.action_scale
        return actions_scaled + self.params.default_dof_pos

    def ComputeOutput(self, actions):
        actions_scaled = actions * self.params.action_scale
        pos_actions_scaled = actions_scaled.clone()
        vel_actions_scaled = torch.zeros_like(actions)
        for i in self.params.wheel_indices:
            pos_actions_scaled[0][i] = 0.0
            vel_actions_scaled[0][i] = actions[0][i]
        all_actions_scaled = pos_actions_scaled + vel_actions_scaled
        output_dof_pos = pos_actions_scaled + self.params.default_dof_pos
        output_dof_vel = vel_actions_scaled
        output_dof_tau = self.params.rl_kp * (all_actions_scaled + self.params.default_dof_pos - self.obs.dof_pos) - self.params.rl_kd * self.obs.dof_vel
        output_dof_tau = torch.clamp(output_dof_tau, -(self.params.torque_limits), self.params.torque_limits)
        return output_dof_pos, output_dof_vel, output_dof_tau

    def QuatRotateInverse(self, q, v, framework):
        if framework == "isaacsim":
            q_w = q[:, 0]
            q_vec = q[:, 1:4]
        elif framework == "isaacgym":
            q_w = q[:, 3]
            q_vec = q[:, 0:3]
        shape = q.shape
        a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
        b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
        c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
        return a - b + c

    def StateController(self, state, command):
        # waiting
        if self.running_state == STATE.STATE_WAITING:
            for i in range(self.params.num_of_dofs):
                command.motor_command.q[i] = state.motor_state.q[i]
            if self.control.control_state == STATE.STATE_POS_GETUP:
                self.control.control_state = STATE.STATE_WAITING
                self.getup_percent = 0.0
                for i in range(self.params.num_of_dofs):
                    self.now_state.motor_state.q[i] = state.motor_state.q[i]
                    self.start_state.motor_state.q[i] = self.now_state.motor_state.q[i]
                self.running_state = STATE.STATE_POS_GETUP
                print("\r\n" + LOGGER.INFO + "Switching to STATE_POS_GETUP")

        # stand up (position control)
        elif self.running_state == STATE.STATE_POS_GETUP:
            if self.getup_percent < 1.0:
                self.getup_percent += 1 / 500.0
                self.getup_percent = min(self.getup_percent, 1.0)
                for i in range(self.params.num_of_dofs):
                    command.motor_command.q[i] = (1 - self.getup_percent) * self.now_state.motor_state.q[i] + self.getup_percent * self.params.default_dof_pos[0][i].item()
                    command.motor_command.dq[i] = 0
                    command.motor_command.kp[i] = self.params.fixed_kp[0][i].item()
                    command.motor_command.kd[i] = self.params.fixed_kd[0][i].item()
                    command.motor_command.tau[i] = 0
                print("\r" + LOGGER.INFO + f"Getting up {self.getup_percent * 100.0:.1f}", end='', flush=True)

            if self.control.control_state == STATE.STATE_RL_INIT:
                self.control.control_state = STATE.STATE_WAITING
                self.running_state = STATE.STATE_RL_INIT
                print("\r\n" + LOGGER.INFO + "Switching to STATE_RL_INIT")

            elif self.control.control_state == STATE.STATE_POS_GETDOWN:
                self.control.control_state = STATE.STATE_WAITING
                self.getdown_percent = 0.0
                for i in range(self.params.num_of_dofs):
                    self.now_state.motor_state.q[i] = state.motor_state.q[i]
                self.running_state = STATE.STATE_POS_GETDOWN
                print("\r\n" + LOGGER.INFO + "Switching to STATE_POS_GETDOWN")

        # init obs and start rl loop
        elif self.running_state == STATE.STATE_RL_INIT:
            if self.getup_percent == 1:
                self.InitObservations()
                self.InitOutputs()
                self.InitControl()
                self.running_state = STATE.STATE_RL_RUNNING
                print("\r\n" + LOGGER.INFO + "Switching to STATE_RL_RUNNING")

        # rl loop
        if self.running_state == STATE.STATE_RL_RUNNING:
            print("\r" + LOGGER.INFO + f"RL Controller x: {self.control.x:.1f} y: {self.control.y:.1f} yaw: {self.control.yaw:.1f}", end='', flush=True)
            for i in range(self.params.num_of_dofs):
                command.motor_command.q[i] = self.output_dof_pos[0][i].item()
                command.motor_command.dq[i] = 0
                command.motor_command.kp[i] = self.params.rl_kp[0][i].item()
                command.motor_command.kd[i] = self.params.rl_kd[0][i].item()
                command.motor_command.tau[i] = 0

            if self.control.control_state == STATE.STATE_POS_GETDOWN:
                self.control.control_state = STATE.STATE_WAITING
                self.getdown_percent = 0.0
                for i in range(self.params.num_of_dofs):
                    self.now_state.motor_state.q[i] = state.motor_state.q[i]
                self.running_state = STATE.STATE_POS_GETDOWN
                print("\r\n" + LOGGER.INFO + "Switching to STATE_POS_GETDOWN")

            elif self.control.control_state == STATE.STATE_POS_GETUP:
                self.control.control_state = STATE.STATE_WAITING
                self.getup_percent = 0.0
                for i in range(self.params.num_of_dofs):
                    self.now_state.motor_state.q[i] = state.motor_state.q[i]
                self.running_state = STATE.STATE_POS_GETUP
                print("\r\n" + LOGGER.INFO + "Switching to STATE_POS_GETUP")

        # get down (position control)
        elif self.running_state == STATE.STATE_POS_GETDOWN:
            if self.getdown_percent < 1.0:
                self.getdown_percent += 1 / 500.0
                self.getdown_percent = min(1.0, self.getdown_percent)
                for i in range(self.params.num_of_dofs):
                    command.motor_command.q[i] = (1 - self.getdown_percent) * self.now_state.motor_state.q[i] + self.getdown_percent * self.start_state.motor_state.q[i]
                    command.motor_command.dq[i] = 0
                    command.motor_command.kp[i] = self.params.fixed_kp[0][i].item()
                    command.motor_command.kd[i] = self.params.fixed_kd[0][i].item()
                    command.motor_command.tau[i] = 0
                print("\r" + LOGGER.INFO + f"Getting down {self.getdown_percent * 100.0:.1f}", end='', flush=True)

            if self.getdown_percent == 1:
                self.InitObservations()
                self.InitOutputs()
                self.InitControl()
                self.running_state = STATE.STATE_WAITING
                print("\r\n" + LOGGER.INFO + "Switching to STATE_WAITING")

    def TorqueProtect(self, origin_output_dof_tau):
        out_of_range_indices = []
        out_of_range_values = []

        for i in range(origin_output_dof_tau.size(1)):
            torque_value = origin_output_dof_tau[0][i].item()
            limit_lower = -self.params.torque_limits[0][i].item()
            limit_upper = self.params.torque_limits[0][i].item()

            if torque_value < limit_lower or torque_value > limit_upper:
                out_of_range_indices.append(i)
                out_of_range_values.append(torque_value)

        if out_of_range_indices:
            for i, index in enumerate(out_of_range_indices):
                value = out_of_range_values[i]
                limit_lower = -self.params.torque_limits[0][index].item()
                limit_upper = self.params.torque_limits[0][index].item()

                print(LOGGER.WARNING + f"Torque({index + 1})={value} out of range({limit_lower}, {limit_upper})")

            # Just a reminder, no protection
            self.control.control_state = STATE.STATE_POS_GETDOWN
            print(LOGGER.INFO + "Switching to STATE_POS_GETDOWN")

    def KeyboardInterface(self, key):
        try:
            if hasattr(key, 'char'):
                if key.char == '0':
                    self.control.control_state = STATE.STATE_POS_GETUP
                elif key.char == 'p':
                    self.control.control_state = STATE.STATE_RL_INIT
                elif key.char == '1':
                    self.control.control_state = STATE.STATE_POS_GETDOWN
                elif key.char == 'w':
                    self.control.x += 0.1
                elif key.char == 's':
                    self.control.x -= 0.1
                elif key.char == 'a':
                    self.control.yaw += 0.1
                elif key.char == 'd':
                    self.control.yaw -= 0.1
                elif key.char == 'j':
                    self.control.y += 0.1
                elif key.char == 'l':
                    self.control.y -= 0.1
                elif key.char == 'r':
                    self.control.control_state = STATE.STATE_RESET_SIMULATION
            else:
                if key == keyboard.Key.enter:
                    self.control.control_state = STATE.STATE_TOGGLE_SIMULATION
                elif key == keyboard.Key.space:
                    self.control.x = 0
                    self.control.y = 0
                    self.control.yaw = 0
        except AttributeError:
            pass


    def ReadYaml(self, robot_path):
        # The config file is located at "rl_sar/src/rl_sar/models/<robot_path>/config.yaml"
        config_path = os.path.join(BASE_PATH, "models", robot_path, "config.yaml")
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)[robot_path]
        except FileNotFoundError as e:
            print(LOGGER.ERROR + f"The file '{config_path}' does not exist")
            return

        self.params.model_name = config["model_name"]
        self.params.framework = config["framework"]
        self.params.dt = config["dt"]
        self.params.decimation = config["decimation"]
        self.params.num_observations = config["num_observations"]
        self.params.observations = config["observations"]
        self.params.observations_history = config["observations_history"]
        self.params.clip_obs = config["clip_obs"]
        if config["clip_actions_lower"] is None and config["clip_actions_upper"] is None:
            self.params.clip_actions_upper = None
            self.params.clip_actions_lower = None
        else:
            self.params.clip_actions_upper = torch.tensor(config["clip_actions_upper"]).view(1, -1)
            self.params.clip_actions_lower = torch.tensor(config["clip_actions_lower"]).view(1, -1)
        self.params.action_scale = torch.tensor(config["action_scale"]).view(1, -1)
        self.params.wheel_indices = config["wheel_indices"]
        self.params.num_of_dofs = config["num_of_dofs"]
        self.params.lin_vel_scale = config["lin_vel_scale"]
        self.params.ang_vel_scale = config["ang_vel_scale"]
        self.params.dof_pos_scale = config["dof_pos_scale"]
        self.params.dof_vel_scale = config["dof_vel_scale"]
        self.params.commands_scale = torch.tensor([self.params.lin_vel_scale, self.params.lin_vel_scale, self.params.ang_vel_scale])
        self.params.rl_kp = torch.tensor(config["rl_kp"]).view(1, -1)
        self.params.rl_kd = torch.tensor(config["rl_kd"]).view(1, -1)
        self.params.fixed_kp = torch.tensor(config["fixed_kp"]).view(1, -1)
        self.params.fixed_kd = torch.tensor(config["fixed_kd"]).view(1, -1)
        self.params.torque_limits = torch.tensor(config["torque_limits"]).view(1, -1)
        self.params.default_dof_pos = torch.tensor(config["default_dof_pos"]).view(1, -1)
        self.params.joint_controller_names = config["joint_controller_names"]
        self.params.command_mapping = config["command_mapping"]
        self.params.state_mapping = config["state_mapping"]

    def CSVInit(self, robot_name):
        self.csv_filename = os.path.join(BASE_PATH, "models", robot_name, 'motor')

        # Uncomment these lines if need timestamp for file name
        # now = datetime.now()
        # timestamp = now.strftime("%Y%m%d%H%M%S")
        # self.csv_filename += f"_{timestamp}"

        self.csv_filename += ".csv"

        with open(self.csv_filename, 'w', newline='') as file:
            writer = csv.writer(file)

            header = []
            header += [f"tau_cal_{i}" for i in range(12)]
            header += [f"tau_est_{i}" for i in range(12)]
            header += [f"joint_pos_{i}" for i in range(12)]
            header += [f"joint_pos_target_{i}" for i in range(12)]
            header += [f"joint_vel_{i}" for i in range(12)]

            writer.writerow(header)

    def CSVLogger(self, torque, tau_est, joint_pos, joint_pos_target, joint_vel):
        with open(self.csv_filename, 'a', newline='') as file:
            writer = csv.writer(file)

            row = []
            row += [torque[0][i].item() for i in range(12)]
            row += [tau_est[0][i].item() for i in range(12)]
            row += [joint_pos[0][i].item() for i in range(12)]
            row += [joint_pos_target[0][i].item() for i in range(12)]
            row += [joint_vel[0][i].item() for i in range(12)]

            writer.writerow(row)
