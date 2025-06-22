import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    rname = LaunchConfiguration("rname")
    cfg = LaunchConfiguration("cfg")

    wname = "stairs"
    robot_name = ParameterValue(Command(["echo -n ", rname]), value_type=str)
    ros_namespace = ParameterValue(Command(["echo -n ", "/", rname, "_gazebo"]), value_type=str)
    gazebo_model_name = ParameterValue(Command(["echo -n ", rname, "_gazebo"]), value_type=str)
    config_name = ParameterValue(Command(["echo -n ", cfg]), value_type=str)

    robot_description = ParameterValue(
        Command([
            "xacro ",
            Command(["echo -n ", Command(["ros2 pkg prefix ", rname, "_description"])]),
            "/share/", rname, "_description/xacro/robot.xacro"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "verbose": "false",
            "world": os.path.join(get_package_share_directory("rl_sar"), "worlds", wname + ".world"),
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "robot_model"],
        output="screen",
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

    # joint_state_broadcaster_node = Node(
    #     package="controller_manager",
    #     executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
    #     arguments=["joint_state_broadcaster"],
    #     output="screen",
    # )

    # robot_joint_controller_node = Node(
    #     package="controller_manager",
    #     executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
    #     arguments=["robot_joint_controller"],
    #     output="screen",
    # )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
            "config_name": config_name,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rname",
            description="Robot name (e.g., a1, go2)",
            default_value=TextSubstitution(text=""),
        ),
        DeclareLaunchArgument(
            "cfg",
            description="Policy cfg",
            default_value=TextSubstitution(text=""),
        ),
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        # joint_state_broadcaster_node,
        # robot_joint_controller_node,
        *controller_nodes,
        param_node,
    ])
