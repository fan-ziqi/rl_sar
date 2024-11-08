import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition

import xacro


def generate_launch_description():

    ####### DATA INPUT ##########
    # urdf_file = 'moonbotX.urdf'
    package_description = "a1_description"

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_description))
    xacro_file = os.path.join(pkg_path,'xacro','robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    jsp_arg = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='True'
    )

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        output='screen',
        parameters=[params]
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=IfCondition(use_joint_state_publisher)
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(
        get_package_share_directory(package_description),
        'launch',
        'check_joint.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        jsp_arg,

        # map_publisher_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node

    ])