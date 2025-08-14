#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('rl_sar')
    
    # Declare launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'launch', 'visualization.rviz'),
        description='Path to RViz config file'
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )
    
    # Static transform publishers
    # Transform from world to base_link
    static_tf_world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen'
    )
    
    # Transform from base_link to sensor_frame (example)
    static_tf_base_to_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_sensor',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'sensor_frame'],
        output='screen'
    )
    
    # Object visualizer node
    object_visualizer_node = Node(
        package='rl_sar',
        executable='object_visualizaer.py',
        name='object_visualizer',
        output='screen'
    )

    # Interactive visualizer node
    Interactive_visualizer_node = Node(
        package='rl_sar',
        executable='interactive_marker.py',
        name='interactive_visualizer',
        output='screen'
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node,
        static_tf_world_to_base,
        static_tf_base_to_sensor,
        object_visualizer_node,
    ])
