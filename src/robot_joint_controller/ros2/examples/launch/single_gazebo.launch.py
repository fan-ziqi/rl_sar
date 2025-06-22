import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    package_name = 'a1_description'
    package_path = os.path.join(get_package_share_directory(package_name))

    xacro_file = os.path.join(package_path, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'false'}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'a1_gazebo'],
        output='screen',
    )

    controller_names = [
        "joint_state_broadcaster",
        "FL_hip_controller", "FL_thigh_controller", "FL_calf_controller",
        "FR_hip_controller", "FR_thigh_controller", "FR_calf_controller",
        "RL_hip_controller", "RL_thigh_controller", "RL_calf_controller",
        "RR_hip_controller", "RR_thigh_controller", "RR_calf_controller"
    ]

    controller_nodes = [Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=[name],
        output='screen',
    ) for name in controller_names]

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        *controller_nodes,
    ])
