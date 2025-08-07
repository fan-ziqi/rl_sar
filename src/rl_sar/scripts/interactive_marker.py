#! /usr/bin/env  python3

import copy
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker


class SingleMarkerBroadcaster(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')
        self.server = InteractiveMarkerServer(self, "target_pose")
        self.pose = PoseStamped()

        self.pose_pub = self.create_publisher(PoseStamped, '/desired_goal_pose', 1)

        self.global_frame = 'world'

    def makeBox(self, msg):
        marker = Marker()

        marker.type = Marker.SPHERE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.5
        return marker

    def create_marker(self):
        self.int_marker = InteractiveMarker()
        int_marker = self.int_marker
        int_marker.header.frame_id = self.global_frame
        int_marker.header.stamp = self.get_clock().now().to_msg()
        self.pose.header = int_marker.header

        int_marker.pose.position.x = 1.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.8
        self.pose.pose.position = int_marker.pose.position

        int_marker.pose.orientation.x = 0.0
        int_marker.pose.orientation.y = 0.0
        int_marker.pose.orientation.z = 0.0
        int_marker.pose.orientation.w = 1.0
        self.pose.pose.orientation = int_marker.pose.orientation

        int_marker.scale = 0.2

        int_marker.name = "PoseTarget"
        int_marker.description = "Pose target for the 3D Goal"

        control = InteractiveMarkerControl()

        # Custom move on plane
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.name = "click"
        control.markers.append(self.makeBox(int_marker))
        int_marker.controls.append(copy.deepcopy(control))

        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.update_pose_callback)

    def apply_changes(self):
        self.server.applyChanges()

    def update_pose_callback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.pose.header.frame_id = self.global_frame
            self.pose.header.stamp = self.get_clock().now().to_msg()
            self.pose.pose.position = feedback.pose.position
            self.pose.pose.orientation = feedback.pose.orientation
        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.get_logger().info("Marker clicked, publish new goal!")
            self.pose_pub.publish(self.pose)


if __name__ == '__main__':
    rclpy.init()
    interactiveTargetPose = None

    try:
        interactiveTargetPose = SingleMarkerBroadcaster()
        interactiveTargetPose.create_marker()
        interactiveTargetPose.apply_changes()
        print(" InteractiveMarker Init Successful ! ")
        
        rclpy.spin(interactiveTargetPose)
    except Exception as e:
        print(f" InteractiveMarker Init Failed ! Error: {e}")
    finally:
        if interactiveTargetPose is not None:
            interactiveTargetPose.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
