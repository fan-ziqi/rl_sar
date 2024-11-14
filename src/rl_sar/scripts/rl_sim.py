#!/usr/bin/env python3
import os
import torch
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from robot_msgs.msg import RobotCommand as RobotCommandMsg
from robot_msgs.msg import RobotState as RobotStateMsg
from robot_msgs.msg import MotorCommand as MotorCommandMsg
from robot_msgs.msg import MotorState as MotorStateMsg
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory
from rl_sdk import *
from observation_buffer import *

CSV_LOGGER = False

class RL_Sim(RL, Node):
    def __init__(self):
        Node.__init__(self, "rl_sim_node")
        RL.__init__(self)

        # member variables for RL_Sim
        self.cmd_vel = Twist()
        self.gazebo_imu = Imu()
        self.robot_state_subscriber_msg = RobotStateMsg()
        self.robot_command_publisher_msg = RobotCommandMsg()

        self.ros_namespace = self.get_namespace()

        # get params from param_node
        self.param_client = self.create_client(GetParameters, '/param_node/get_parameters')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for param_node service to be available...")
        request = GetParameters.Request()
        request.names = ['robot_name', 'gazebo_model_name']
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if len(future.result().values) < 2:
            self.get_logger().warn("Failed to get all parameters from param_node")
        else:
            self.robot_name = future.result().values[0].string_value
            self.gazebo_model_name = future.result().values[1].string_value
            self.get_logger().info(f"Get param robot_name: {self.robot_name}")
            self.get_logger().info(f"Get param gazebo_model_name: {self.gazebo_model_name}")

        # read params from yaml
        self.ReadYaml(self.robot_name)
        for i in range(len(self.params.observations)):
            if self.params.observations[i] == "ang_vel":
                self.params.observations[i] = "ang_vel_body"

        # init rl
        torch.set_grad_enabled(False)
        if len(self.params.observations_history) != 0:
            self.history_obs_buf = ObservationBuffer(1, self.params.num_observations, len(self.params.observations_history))
        self.robot_command_publisher_msg.motor_command = [MotorCommandMsg() for _ in range(self.params.num_of_dofs)]
        self.robot_state_subscriber_msg.motor_state = [MotorStateMsg() for _ in range(self.params.num_of_dofs)]
        self.InitObservations()
        self.InitOutputs()
        self.InitControl()
        self.running_state = STATE.STATE_RL_RUNNING

        # model
        model_path = os.path.join(get_package_share_directory('rl_sar'), 'models', self.robot_name, self.params.model_name)
        self.model = torch.jit.load(model_path)

        # publisher
        self.robot_command_publisher = self.create_publisher(RobotCommandMsg, self.ros_namespace + "robot_joint_controller/command", qos_profile_system_default)

        # subscriber
        self.cmd_vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.CmdvelCallback, qos_profile_system_default)
        self.gazebo_imu_subscriber = self.create_subscription(Imu, "/imu", self.GazeboImuCallback, qos_profile_system_default)
        self.robot_state_subscriber = self.create_subscription(RobotStateMsg, self.ros_namespace + "robot_joint_controller/state", self.RobotStateCallback, qos_profile_system_default)

        # service
        self.gazebo_set_model_state_client = self.create_client(SetModelState, "/gazebo/set_model_state")
        self.gazebo_pause_physics_client = self.create_client(Empty, "/gazebo/pause_physics")
        self.gazebo_unpause_physics_client = self.create_client(Empty, "/gazebo/unpause_physics")

        # loops
        self.thread_control = threading.Thread(target=self.ThreadControl)
        self.thread_rl = threading.Thread(target=self.ThreadRL)
        self.thread_control.daemon = True
        self.thread_rl.daemon = True
        self.thread_control.start()
        self.thread_rl.start()

        # keyboard
        self.listener_keyboard = keyboard.Listener(on_press=self.KeyboardInterface)
        self.listener_keyboard.start()

        # others
        if CSV_LOGGER:
            self.CSVInit(self.robot_name)

        self.get_logger().info("RL_Sim start")

    def __del__(self):
        self.get_logger().info("RL_Sim exit")

    def GetState(self, state):
        if self.params.framework == "isaacgym":
            state.imu.quaternion[3] = self.gazebo_imu.orientation.w
            state.imu.quaternion[0] = self.gazebo_imu.orientation.x
            state.imu.quaternion[1] = self.gazebo_imu.orientation.y
            state.imu.quaternion[2] = self.gazebo_imu.orientation.z
        elif self.params.framework == "isaacsim":
            state.imu.quaternion[0] = self.gazebo_imu.orientation.w
            state.imu.quaternion[1] = self.gazebo_imu.orientation.x
            state.imu.quaternion[2] = self.gazebo_imu.orientation.y
            state.imu.quaternion[3] = self.gazebo_imu.orientation.z

        state.imu.gyroscope[0] = self.gazebo_imu.angular_velocity.x
        state.imu.gyroscope[1] = self.gazebo_imu.angular_velocity.y
        state.imu.gyroscope[2] = self.gazebo_imu.angular_velocity.z

        state.imu.accelerometer[0] = self.gazebo_imu.linear_acceleration.x
        state.imu.accelerometer[1] = self.gazebo_imu.linear_acceleration.y
        state.imu.accelerometer[2] = self.gazebo_imu.linear_acceleration.z

        for i in range(self.params.num_of_dofs):
            state.motor_state.q[i] = self.robot_state_subscriber_msg.motor_state[i].q
            state.motor_state.dq[i] = self.robot_state_subscriber_msg.motor_state[i].dq
            state.motor_state.tau_est[i] = self.robot_state_subscriber_msg.motor_state[i].tau_est

    def SetCommand(self, command):
        for i in range(self.params.num_of_dofs):
            self.robot_command_publisher_msg.motor_command[i].q = float(command.motor_command.q[i])
            self.robot_command_publisher_msg.motor_command[i].dq = float(command.motor_command.dq[i])
            self.robot_command_publisher_msg.motor_command[i].kp = float(command.motor_command.kp[i])
            self.robot_command_publisher_msg.motor_command[i].kd = float(command.motor_command.kd[i])
            self.robot_command_publisher_msg.motor_command[i].tau = float(command.motor_command.tau[i])

        self.robot_command_publisher.publish(self.robot_command_publisher_msg)

    def RobotControl(self):
        # NOT AVAILABLE NOW
        # if self.control.control_state == STATE.STATE_RESET_SIMULATION:
        #     set_model_state = SetModelState.Request()
        #     set_model_state.model_state.model_name = self.gazebo_model_name
        #     set_model_state.model_state.pose.position.z = 1.0
        #     set_model_state.model_state.reference_frame = "world"
        #     self.gazebo_set_model_state_client.call_async(set_model_state)
        #     self.control.control_state = STATE.STATE_WAITING
        # if self.control.control_state == STATE.STATE_TOGGLE_SIMULATION:
        #     if self.simulation_running:
        #         self.gazebo_pause_physics_client.call_async(Empty.Request())
        #         self.get_logger().info("Simulation Stop")
        #     else:
        #         self.gazebo_unpause_physics_client.call_async(Empty.Request())
        #         self.get_logger().info("Simulation Start")
        #     self.simulation_running = not self.simulation_running
        #     self.control.control_state = STATE.STATE_WAITING

        if self.simulation_running:
            self.GetState(self.robot_state)
            self.StateController(self.robot_state, self.robot_command)
            self.SetCommand(self.robot_command)

    def GazeboImuCallback(self, msg):
        self.gazebo_imu = msg

    def CmdvelCallback(self, msg):
        self.cmd_vel = msg

    def RobotStateCallback(self, msg):
        self.robot_state_subscriber_msg = msg

    def RunModel(self):
        if self.running_state == STATE.STATE_RL_RUNNING and self.simulation_running:
            # self.obs.lin_vel NOT USE
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

            if CSV_LOGGER:
                tau_est = torch.empty(1, self.params.num_of_dofs, dtype=torch.float32)
                for i in range(self.params.num_of_dofs):
                    tau_est[0, i] = self.robot_state_subscriber_msg.motor_state[i].tau_est
                self.CSVLogger(self.output_torques, tau_est, self.obs.dof_pos, self.output_dof_pos, self.obs.dof_vel)

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
        self.get_logger().info(f"[Thread Start] named: {thread_name}, period: {thread_period * 1000:.0f}(ms), cpu unspecified")
        while rclpy.ok():
            self.RobotControl()
            time.sleep(thread_period)
        self.get_logger().info("[Thread End] named: " + thread_name)

    def ThreadRL(self):
        thread_period = self.params.dt * self.params.decimation
        thread_name = "thread_rl"
        self.get_logger().info(f"[Thread Start] named: {thread_name}, period: {thread_period * 1000:.0f}(ms), cpu unspecified")
        while rclpy.ok():
            self.RunModel()
            time.sleep(thread_period)
        self.get_logger().info("[Thread End] named: " + thread_name)

if __name__ == "__main__":
    rclpy.init()
    rl_sim = RL_Sim()
    try:
        rclpy.spin(rl_sim)
    except KeyboardInterrupt:
        rl_sim.get_logger().info("Shutdown signal received, shutting down...")
        rclpy.shutdown()

