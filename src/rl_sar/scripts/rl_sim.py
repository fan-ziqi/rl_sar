import sys
import os
import torch
import threading
import time
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose
from robot_msgs.msg import MotorCommand
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_srvs.srv import Empty

path = os.path.abspath(".")
sys.path.insert(0, path + "/src/rl_sar/scripts")
from rl_sdk import *
from observation_buffer import *

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
        self.ReadYaml(self.robot_name)

        # history
        self.use_history = rospy.get_param("use_history", "")
        if self.use_history:
            self.history_obs_buf = ObservationBuffer(1, self.params.num_observations, 6)

        # Due to the fact that the robot_state_publisher sorts the joint names alphabetically,
        # the mapping table is established according to the order defined in the YAML file
        sorted_joint_controller_names = sorted(self.params.joint_controller_names)
        self.sorted_to_original_index = {}
        for i in range(len(self.params.joint_controller_names)):
            self.sorted_to_original_index[sorted_joint_controller_names[i]] = i
        self.mapped_joint_positions = [0.0] * self.params.num_of_dofs
        self.mapped_joint_velocities = [0.0] * self.params.num_of_dofs
        self.mapped_joint_efforts = [0.0] * self.params.num_of_dofs

        # init
        torch.set_grad_enabled(False)
        self.joint_publishers_commands = [MotorCommand() for _ in range(self.params.num_of_dofs)]
        self.InitObservations()
        self.InitOutputs()
        self.InitControl()

        # model
        model_path = os.path.join(os.path.dirname(__file__), f"../models/{self.robot_name}/{self.params.model_name}")
        self.model = torch.jit.load(model_path)

        # publisher
        self.ros_namespace = rospy.get_param("ros_namespace", "")
        self.joint_publishers = {}
        for i in range(self.params.num_of_dofs):
            topic_name = f"{self.ros_namespace}{self.params.joint_controller_names[i]}/command"
            self.joint_publishers[self.params.joint_controller_names[i]] = rospy.Publisher(topic_name, MotorCommand, queue_size=10)

        # subscriber
        self.cmd_vel_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.CmdvelCallback, queue_size=10)
        self.model_state_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.ModelStatesCallback, queue_size=10)
        joint_states_topic = f"{self.ros_namespace}joint_states"
        self.joint_state_subscriber = rospy.Subscriber(joint_states_topic, JointState, self.JointStatesCallback, queue_size=10)

        # service
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

        print(LOGGER.INFO + "RL_Sim start")

    def __del__(self):
        print(LOGGER.INFO + "RL_Sim exit")

    def GetState(self, state):
        state.imu.quaternion[3] = self.pose.orientation.w
        state.imu.quaternion[0] = self.pose.orientation.x
        state.imu.quaternion[1] = self.pose.orientation.y
        state.imu.quaternion[2] = self.pose.orientation.z

        state.imu.gyroscope[0] = self.vel.angular.x
        state.imu.gyroscope[1] = self.vel.angular.y
        state.imu.gyroscope[2] = self.vel.angular.z

        # state.imu.accelerometer

        for i in range(self.params.num_of_dofs):
            state.motor_state.q[i] = self.mapped_joint_positions[i]
            state.motor_state.dq[i] = self.mapped_joint_velocities[i]
            state.motor_state.tauEst[i] = self.mapped_joint_efforts[i]

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
            gazebo_model_name = f"{self.robot_name}_gazebo"
            set_model_state.model_name = gazebo_model_name
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

    def MapData(self, source_data, target_data):
        for i in range(len(source_data)):
            target_data[i] = source_data[self.sorted_to_original_index[self.params.joint_controller_names[i]]]

    def JointStatesCallback(self, msg):
        self.MapData(msg.position, self.mapped_joint_positions)
        self.MapData(msg.velocity, self.mapped_joint_velocities)
        self.MapData(msg.effort, self.mapped_joint_efforts)

    def RunModel(self):
        if self.running_state == STATE.STATE_RL_RUNNING and self.simulation_running:
            # self.obs.lin_vel = torch.tensor([[self.vel.linear.x, self.vel.linear.y, self.vel.linear.z]])
            self.obs.ang_vel = torch.tensor(self.robot_state.imu.gyroscope).unsqueeze(0)
            # self.obs.commands = torch.tensor([[self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.angular.z]])
            self.obs.commands = torch.tensor([[self.control.x, self.control.y, self.control.yaw]])
            self.obs.base_quat = torch.tensor(self.robot_state.imu.quaternion).unsqueeze(0)
            self.obs.dof_pos = torch.tensor(self.robot_state.motor_state.q).narrow(0, 0, self.params.num_of_dofs).unsqueeze(0)
            self.obs.dof_vel = torch.tensor(self.robot_state.motor_state.dq).narrow(0, 0, self.params.num_of_dofs).unsqueeze(0)

            clamped_actions = self.Forward()

            for i in self.params.hip_scale_reduction_indices:
                clamped_actions[0][i] *= self.params.hip_scale_reduction

            self.obs.actions = clamped_actions

            origin_output_torques = self.ComputeTorques(self.obs.actions)

            # self.TorqueProtect(origin_output_torques)

            self.output_torques = torch.clamp(origin_output_torques, -(self.params.torque_limits), self.params.torque_limits)
            self.output_dof_pos = self.ComputePosition(self.obs.actions)

    def ComputeObservation(self):
        obs = torch.cat([
            # self.obs.lin_vel * self.params.lin_vel_scale,
            # self.obs.ang_vel * self.params.ang_vel_scale, # TODO is QuatRotateInverse necessery?
            self.QuatRotateInverse(self.obs.base_quat, self.obs.ang_vel) * self.params.ang_vel_scale,
            self.QuatRotateInverse(self.obs.base_quat, self.obs.gravity_vec),
            self.obs.commands * self.params.commands_scale,
            (self.obs.dof_pos - self.params.default_dof_pos) * self.params.dof_pos_scale,
            self.obs.dof_vel * self.params.dof_vel_scale,
            self.obs.actions
        ], dim = -1)
        clamped_obs = torch.clamp(obs, -self.params.clip_obs, self.params.clip_obs)
        return clamped_obs

    def Forward(self):
        torch.set_grad_enabled(False)
        clamped_obs = self.ComputeObservation()
        if self.use_history:
            self.history_obs_buf.insert(clamped_obs)
            history_obs = self.history_obs_buf.get_obs_vec(np.arange(6))
            actions = self.model.forward(history_obs)
        else:
            actions = self.model.forward(clamped_obs)
        clamped_actions = torch.clamp(actions, self.params.clip_actions_lower, self.params.clip_actions_upper)
        return clamped_actions

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
    