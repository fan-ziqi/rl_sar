# 导入操作系统接口模块（用于路径操作）
import os
# 导入ROS2启动描述核心模块
from launch import LaunchDescription
# 导入启动动作：包含其他启动文件、声明启动参数
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# 导入Python启动描述源（用于动态加载其他launch文件）
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 导入动态配置替换工具：启动配置、文本替换、命令行输出
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
# 导入ROS节点操作类
from launch_ros.actions import Node
# 导入参数值描述类（确保类型安全）
from launch_ros.parameter_descriptions import ParameterValue
# 导入功能包资源路径定位工具
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 引用动态参数：机器人型号（如a1/go2）
    rname = LaunchConfiguration("rname")
    # 引用动态参数：框架类型（如isaacgym/isaacsim）
    framework = LaunchConfiguration("framework")

    # 固定世界名称（此处使用楼梯场景）
    wname = "stair"

    # 动态生成完整机器人名称（拼接型号与框架）
    robot_name = ParameterValue(Command(["echo -n ", rname, "_", framework]), value_type=str)
    # 动态生成ROS命名空间（格式：/机器人型号_gazebo）
    ros_namespace = ParameterValue(Command(["echo -n ", "/", rname, "_gazebo"]), value_type=str)
    # 动态生成Gazebo模型名称（格式：机器人型号_gazebo）
    gazebo_model_name = ParameterValue(Command(["echo -n ", rname, "_gazebo"]), value_type=str)

    # 动态构建机器人URDF描述（通过xacro解析）
    robot_description = ParameterValue(
        Command([
            "xacro ",
            # 定位功能包安装路径（如/opt/ros/humble/share/go2_description）
            Command(["echo -n ", Command(["ros2 pkg prefix ", rname, "_description"])]),
            # 拼接Xacro文件路径（标准路径结构：<包路径>/share/<包名>/xacro/robot.xacro）
            "/share/", rname, "_description/xacro/robot.xacro"
        ]),
        value_type=str
    )

    # 机器人状态发布节点（将URDF发布到TF）
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # 定位gazebo_ros包的启动文件路径
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "verbose": "false",      # 关闭冗余输出
            # 加载指定世界文件（从rl_sar包的世界目录获取）
            "world": os.path.join(get_package_share_directory("rl_sar"), "worlds", wname + ".world"),
        }.items(),
    )

    # 在Gazebo中生成机器人实体
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description",      #订阅URDF话题
                    "-entity", "robot_model"],          #指定模型名称
        output="screen",
    )

    # 启动关节状态广播控制器（发布/joint_states）[4](@ref)
    joint_state_broadcaster_node = Node(
        package="controller_manager",

        # ROS版本适配（Foxy使用spawner.py，其他用spawner）
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["joint_state_broadcaster"],      #控制器名称
        output="screen",
    )

    # 启动机器人关节运动控制器（如PID控制器）
    robot_joint_controller_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["robot_joint_controller"],       # 自定义控制器名称
        output="screen",
    )

    # 参数服务器节点（存储全局参数）
    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
        }],
    )

     # 构建最终启动描述
    return LaunchDescription([
        # 声明必须参数：机器人型号（启动时需通过命令行指定）
        DeclareLaunchArgument(
            "rname",
            description="Robot name (e.g., a1, go2)",
            default_value=TextSubstitution(text=""),    # 默认空值强制用户指定
        ),

        # 声明必须参数：仿真框架类型
        DeclareLaunchArgument(
            "framework",
            description="Framework (isaacgym or isaacsim)",
            default_value=TextSubstitution(text=""),
        ),

        # 按顺序启动的节点列表
        robot_state_publisher_node,         # 1. 发布机器人模型
        gazebo,                             # 2. 启动Gazebo
        spawn_entity,                       # 3. 加载机器人到Gazebo
        joint_state_broadcaster_node,       # 4. 激活关节状态广播
        robot_joint_controller_node,        # 5. 激活关节控制器
        param_node,                         # 6. 存储全局参数
    ])
