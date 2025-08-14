#! /usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import MarkerArray, Marker

BASE_PATH = os.path.join(os.path.dirname(__file__), "../")
# BASE_PATH = os.path.join(os.path.dirname(__file__))

class ObjectVisualizer(Node):
    def __init__(self):
        super().__init__('object_visualizer')
        self.get_logger().info('Initializing object visualizer')
        
        try :
            visualizer_path = get_package_share_directory('rl_sar')
            self.visualizer_path = os.path.join(visualizer_path, 'worlds')
        except:
            self.visualizer_path = os.path.join(BASE_PATH, 'worlds')
            
        self.markerArray = MarkerArray()
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker', 1)
        
        self.timer = self.create_timer(0.2, self.timer_callback)  # 0.2 Hz
        self.current_file_count = 0

        if not os.path.exists(self.visualizer_path):
            self.get_logger().warn(f'Meshes directory does not exist: {self.visualizer_path}')
            return
        
        self.file = "benchmark_bin_centered_resized.stl"
        # self.files = [file for file in os.listdir(self.visualizer_path) if file.endswith(('.stl','.STL'))]  # find all meshes in the 'meshes' folder

        # for marker_id, file in enumerate(self.files):
            # if os.path.exists(os.path.join(BASE_PATH, "worlds", file)):
        if os.path.exists(self.visualizer_path + "/"  + self.file):
            self.get_logger().info(f'Published object : {self.file}')

    def timer_callback(self):        

        self.markerArray.markers.clear()  # Clear previous markers
        # for marker_id, file in enumerate(self.files):
            # self.get_logger().info(f'Loading file: {file}')
        marker = Marker()
        marker.id = 2
        marker.mesh_resource = 'file://' + os.path.join(self.visualizer_path, self.file)
        # marker.mesh_resource = self.visualizer_path + "/"  + file
        marker.mesh_use_embedded_materials = True  # Need this to use textures for mesh
        marker.type = Marker.MESH_RESOURCE
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0  # Fully opaque
        marker.color.r = 0.2  # Red
        marker.color.g = 0.2  # Green
        marker.color.b = 0.2
        self.markerArray.markers.append(marker)
        self.publisher.publish(self.markerArray)

def main(args=None):
    rclpy.init(args=args)
    object_visualizer = ObjectVisualizer()

    rclpy.spin(object_visualizer)

    object_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()