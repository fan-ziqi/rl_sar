# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import sys
import os
import torch
import threading
import time
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Pose
from robot_msgs.msg import MotorState, MotorCommand
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_srvs.srv import Empty

path = os.path.abspath(".")
sys.path.insert(0, path + "/src/rl_sar/scripts")
from rl_sdk import *
from observation_buffer import *

CSV_LOGGER = False

class RL_Sim(RL):
    def __init__(self):
        super().__init__()

        # member variables for RL_Sim
        self.vel = Twist()
        self.pose = Pose()
        self.cmd_vel = Twist()

        # start ros node
        rospy.init_node("rl_sim")

        # read params from yaml
        self.robot_name = rospy.get_param("robot_name", "")
        self.config_name = rospy.get_param("config_name", "")
        robot_path = self.robot_name + "/" + self.config_name
        self.ReadYaml(robot_path)
        for i in range(len(self.params.observations)):
            if self.params.observations[i] == "ang_vel":
                self.params.observations[i] = "ang_vel_world"

        # history
        if len(self.params.observations_history) != 0:
            self.history_obs_buf = ObservationBuffer(1, self.params.num_observations, len(self.params.observations_history))

        # init
        torch.set_grad_enabled(False)
        torch.set_num_threads(4)
        self.joint_publishers_commands = [MotorCommand() for _ in range(self.params.num_of_dofs)]
        self.InitObservations()
        self.InitOutputs()
        self.InitControl()
        self.running_state = STATE.STATE_RL_RUNNING

        # model
        model_path = os.path.join(os.path.dirname(__file__), f"../models/{robot_path}/{self.params.model_name}")
        self.model = torch.jit.load(model_path)

        # publisher
        self.ros_namespace = rospy.get_param("ros_namespace", "")
        self.joint_publishers = {}
        for i in range(self.params.num_of_dofs):
            joint_name = self.params.joint_controller_names[i]
            topic_name = f"{self.ros_namespace}{joint_name}/command"
            self.joint_publishers[self.params.joint_controller_names[i]] = rospy.Publisher(topic_name, MotorCommand, queue_size=10)

        # subscriber
        self.cmd_vel_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.CmdvelCallback, queue_size=10)
        self.model_state_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.ModelStatesCallback, queue_size=10)
        self.joint_subscribers = {}
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_efforts = {}
        for i in range(self.params.num_of_dofs):
            joint_name = self.params.joint_controller_names[i]
            topic_name = f"{self.ros_namespace}{joint_name}/state"
            self.joint_subscribers[joint_name] = rospy.Subscriber(
                topic_name,
                MotorState,
                lambda msg, name=joint_name: self.JointStatesCallback(msg, name),
                queue_size=10
            )
            self.joint_positions[joint_name] = 0.0
            self.joint_velocities[joint_name] = 0.0
            self.joint_efforts[joint_name] = 0.0

        # service
        self.gazebo_model_name = rospy.get_param("gazebo_model_name", "")
        self.gazebo_set_model_state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.gazebo_pause_physics_client = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.gazebo_unpause_physics_client = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

        # loops
        self.thread_control = threading.Thread(target=self.ThreadControl)
        self.thread_rl = threading.Thread(target=self.ThreadRL)
        self.thread_control.start()
        self.thread_rl.start()

        # keyboard
        self.listener_keyboard = keyboard.Listener(on_press=self.KeyboardInterface)
        self.listener_keyboard.start()

        # others
        if CSV_LOGGER:
            self.CSVInit(self.robot_name)

        print(LOGGER.INFO + "RL_Sim start")

    def __del__(self):
        print(LOGGER.INFO + "RL_Sim exit")

    def GetState(self, state):
        if self.params.framework == "isaacgym":
            state.imu.quaternion[3] = self.pose.orientation.w
            state.imu.quaternion[0] = self.pose.orientation.x
            state.imu.quaternion[1] = self.pose.orientation.y
            state.imu.quaternion[2] = self.pose.orientation.z
        elif self.params.framework == "isaacsim":
            state.imu.quaternion[0] = self.pose.orientation.w
            state.imu.quaternion[1] = self.pose.orientation.x
            state.imu.quaternion[2] = self.pose.orientation.y
            state.imu.quaternion[3] = self.pose.orientation.z

        state.imu.gyroscope[0] = self.vel.angular.x
        state.imu.gyroscope[1] = self.vel.angular.y
        state.imu.gyroscope[2] = self.vel.angular.z

        # state.imu.accelerometer

        for i in range(self.params.num_of_dofs):
            state.motor_state.q[i] = self.joint_positions[self.params.joint_controller_names[i]]
            state.motor_state.dq[i] = self.joint_velocities[self.params.joint_controller_names[i]]
            state.motor_state.tau_est[i] = self.joint_efforts[self.params.joint_controller_names[i]]

    def SetCommand(self, command):
        for i in range(self.params.num_of_dofs):
            self.joint_publishers_commands[i].q = command.motor_command.q[i]
            self.joint_publishers_commands[i].dq = command.motor_command.dq[i]
            self.joint_publishers_commands[i].kp = command.motor_command.kp[i]
            self.joint_publishers_commands[i].kd = command.motor_command.kd[i]
            self.joint_publishers_commands[i].tau = command.motor_command.tau[i]

        for i in range(self.params.num_of_dofs):
            self.joint_publishers[self.params.joint_controller_names[i]].publish(self.joint_publishers_commands[i])

    def RobotControl(self):
        if self.control.control_state == STATE.STATE_RESET_SIMULATION:
            set_model_state = SetModelStateRequest().model_state
            set_model_state.model_name = self.gazebo_model_name
            set_model_state.pose.position.z = 1.0
            set_model_state.reference_frame = "world"
            self.gazebo_set_model_state_client(set_model_state)
            self.control.control_state = STATE.STATE_WAITING
        if self.control.control_state == STATE.STATE_TOGGLE_SIMULATION:
            if self.simulation_running:
                self.gazebo_pause_physics_client()
                print("\r\n" + LOGGER.INFO + "Simulation Stop")
            else:
                self.gazebo_unpause_physics_client()
                print("\r\n" + LOGGER.INFO + "Simulation Start")
            self.simulation_running = not self.simulation_running
            self.control.control_state = STATE.STATE_WAITING

        if self.simulation_running:
            self.GetState(self.robot_state)
            self.StateController(self.robot_state, self.robot_command)
            self.SetCommand(self.robot_command)

    def ModelStatesCallback(self, msg):
        self.vel = msg.twist[2]
        self.pose = msg.pose[2]

    def CmdvelCallback(self, msg):
        self.cmd_vel = msg

    def JointStatesCallback(self, msg, joint_name):
        self.joint_positions[joint_name] = msg.q
        self.joint_velocities[joint_name] = msg.dq
        self.joint_efforts[joint_name] = msg.tau_est

    def RunModel(self):
        if self.running_state == STATE.STATE_RL_RUNNING and self.simulation_running:
            self.obs.lin_vel = torch.tensor([[self.vel.linear.x, self.vel.linear.y, self.vel.linear.z]])
            self.obs.ang_vel = torch.tensor(self.robot_state.imu.gyroscope).unsqueeze(0)
            # self.obs.commands = torch.tensor([[self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.angular.z]])
            self.obs.commands = torch.tensor([[self.control.x, self.control.y, self.control.yaw]])
            self.obs.base_quat = torch.tensor(self.robot_state.imu.quaternion).unsqueeze(0)
            self.obs.dof_pos = torch.tensor(self.robot_state.motor_state.q).narrow(0, 0, self.params.num_of_dofs).unsqueeze(0)
            self.obs.dof_vel = torch.tensor(self.robot_state.motor_state.dq).narrow(0, 0, self.params.num_of_dofs).unsqueeze(0)

            self.obs.actions = self.Forward()
            self.output_dof_pos, self.output_dof_vel, self.output_dof_tau = self.ComputeOutput(self.obs.actions)

            # self.TorqueProtect(self.output_dof_pos)

            if CSV_LOGGER:
                tau_est = torch.zeros((1, self.params.num_of_dofs))
                for i in range(self.params.num_of_dofs):
                    tau_est[0, i] = self.joint_efforts[self.params.joint_controller_names[i]]
                self.CSVLogger(self.output_dof_tau, tau_est, self.obs.dof_pos, self.output_dof_pos, self.obs.dof_vel)

    def Forward(self):
        torch.set_grad_enabled(False)
        clamped_obs = self.ComputeObservation()
        if len(self.params.observations_history) != 0:
            self.history_obs_buf.insert(clamped_obs)
            history_obs = self.history_obs_buf.get_obs_vec(self.params.observations_history)
            actions = self.model.forward(history_obs)
        else:
            actions = self.model.forward(clamped_obs)
        if self.params.clip_actions_lower is not None and self.params.clip_actions_upper is not None:
            return torch.clamp(actions, self.params.clip_actions_lower, self.params.clip_actions_upper)
        else:
            return actions

    def ThreadControl(self):
        thread_period = self.params.dt
        thread_name = "thread_control"
        print(f"[Thread Start] named: {thread_name}, period: {thread_period * 1000:.0f}(ms), cpu unspecified")
        while not rospy.is_shutdown():
            self.RobotControl()
            time.sleep(thread_period)
        print("[Thread End] named: " + thread_name)

    def ThreadRL(self):
        thread_period = self.params.dt * self.params.decimation
        thread_name = "thread_rl"
        print(f"[Thread Start] named: {thread_name}, period: {thread_period * 1000:.0f}(ms), cpu unspecified")
        while not rospy.is_shutdown():
            self.RunModel()
            time.sleep(thread_period)
        print("[Thread End] named: " + thread_name)

if __name__ == "__main__":
    rl_sim = RL_Sim()
    rospy.spin()
