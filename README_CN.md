# rl_sar

[![Ubuntu 20.04/22.04](https://img.shields.io/badge/Ubuntu-20.04/22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![macOS](https://img.shields.io/badge/macOS-Experimental-orange.svg?logo=apple)](https://www.apple.com/macos/)
[![ROS Noetic](https://img.shields.io/badge/ros-noetic-brightgreen.svg?logo=ros)](https://wiki.ros.org/noetic)
[![ROS2 Foxy/Humble](https://img.shields.io/badge/ros2-foxy/humble-brightgreen.svg?logo=ros)](https://wiki.ros.org/foxy)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-lightgrey.svg?logo=gazebo)](http://gazebosim.org/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.2.7-orange.svg?logo=mujoco)](https://mujoco.org/)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2-0)

[English document](README.md)

本仓库提供了机器人强化学习算法的仿真验证与实物部署框架，适配四足机器人、轮足机器人、人形机器人。"sar"代表"simulation and real"

> 支持**IsaacGym**和**IsaacSim**
>
> 支持**ROS-Noetic**和**ROS2-Foxy/Humble**
>
> 支持**libtorch**和**onnxruntime**
>
> 支持**Linux**和**macOS**(只支持Mujoco仿真)
>
> 支持**Gazebo**和**Mujoco**(部分支持)
>
> 支持**Locomotion**和**Dance**

支持列表：

|Robot Name (rname:=)|Pre-Trained Policy|Gazebo|Mujoco|Real|
|-|-|-|-|-|
|Unitree-A1 (a1)|legged_gym (IsaacGym)|✅|❌|✅|
|Unitree-Go2 (go2)|himloco (IsaacGym)</br>robot_lab (IsaacSim)|✅|✅|✅</br>✅|
|Unitree-Go2W (go2w)|robot_lab (IsaacSim)|✅|✅|✅|
|Unitree-B2 (b2)|robot_lab (IsaacSim)|✅|✅|⚪|
|Unitree-B2W (b2w)|robot_lab (IsaacSim)|✅|✅|⚪|
|Unitree-G1 (g1)|robomimic/locomotion (IsaacGym)</br>robomimic/charleston (IsaacGym)</br>whole_body_tracking/dance_102 (IsaacSim)</br>whole_body_tracking/gangnam_style (IsaacSim)|✅|✅|✅|
|FFTAI-GR1T1 (gr1t1)</br>(Only available on Ubuntu20.04)|legged_gym (IsaacGym)|✅|❌|⚪|
|FFTAI-GR1T2 (gr1t2)</br>(Only available on Ubuntu20.04)|legged_gym (IsaacGym)|✅|❌|⚪|
|zhinao-L4W4 (l4w4)|legged_gym (IsaacGym)|✅|❌|✅|
|Deeprobotics-Lite3 (lite3)|himloco (IsaacGym)|✅|❌|✅|
|DDTRobot-Tita (tita)|robot_lab (IsaacSim)|✅|❌|⚪|

> [!IMPORTANT]
> Python版本暂时停止维护，如有需要请使用[v2.3](https://github.com/fan-ziqi/rl_sar/releases/tag/v2.3)版本，后续可能会重新上线。

> [!NOTE]
> 如果你想使用IsaacLab（IsaacSim）训练策略，请使用 [robot_lab](https://github.com/fan-ziqi/robot_lab) 项目。
>
> robot_lab配置文件中的关节顺序 `joint_names` 与本项目代码中 `xxx/robot_lab/config.yaml` 中定义的相同。
>
> 在 [Github Discussion](https://github.com/fan-ziqi/rl_sar/discussions) 或 [Discord](https://www.robotsfan.com/dc_rl_sar) 中讨论

> [!CAUTION]
> **免责声明：使用者确认使用本代码产生的所有风险及后果均由使用者自行承担，作者不承担任何直接或间接责任，操作前必须确保已采取充分安全防护措施。**

## 准备

拉取仓库

```bash
git clone --recursive --depth 1 https://github.com/fan-ziqi/rl_sar.git
```

如需更新

```bash
git pull
git submodule update --init --recursive --recommend-shallow --progress
```

## 依赖

安装必要的依赖：

```bash
# Ubuntu
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev

# macOS
brew install boost lcm yaml-cpp tbb libomp pkg-config glfw
```

如果需要使用ROS，安装下列依赖包：

```bash
# ros-noetic (Ubuntu20.04)
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-controller-manager

# ros2-foxy (Ubuntu20.04) / ros2-humble (Ubuntu22.04)
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros2-control ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-xacro
```

## 编译

在项目根目录中执行下面的脚本编译整个项目

```bash
./build.sh
```

若想单独编译某几个包，可以在后面加上包名

```bash
./build.sh package1 package2
```

若想删除构建，可以使用下列命令，此命令会删除所有编译产物和创建的软链接

```bash
./build.sh -c  # or ./build.sh --clean
```

如果不需要仿真，只在机器人上运行，可以使用CMake进行编译，同时禁用ROS（编译生成的可执行文件在`cmake_build/bin`中，库在`cmake_build/lib`中）

```bash
./build.sh -m  # or ./build.sh --cmake
```

若想使用Mujoco仿真器

```bash
./build.sh -mj  # or ./build.sh --mujoco
```

详细的使用说明可以通过`./build.sh -h`查看

```bash
Usage: ./build.sh [OPTIONS] [PACKAGE_NAMES...]

Options:
  -c, --clean    Clean workspace (remove symlinks and build artifacts)
  -m, --cmake    Build using CMake (for hardware deployment only)
  -mj,--mujoco   Build with MuJoCo simulator support (CMake only)"
  -h, --help     Show this help message

Examples:
  ./build.sh                    # Build all ROS packages
  ./build.sh package1 package2  # Build specific ROS packages
  ./build.sh -c                 # Clean all symlinks and build artifacts
  ./build.sh --clean package1   # Clean specific package and build artifacts
  ./build.sh -m                 # Build with CMake for hardware deployment
  ./build.sh -mj                # Build with CMake and MuJoCo simulator support
```

> [!TIP]
> 如果 catkin build 报错: `Unable to find either executable 'empy' or Python module 'em'`, 在`catkin build` 之前执行 `catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3`

## 运行

下文中使用 **\<ROBOT\>/\<CONFIG\>** 代替表示不同的环境，如 `go2/himloco` 、 `go2w/robot_lab`。

运行前请将训练好的pt模型文件拷贝到`rl_sar/src/rl_sar/policy/<ROBOT>/<CONFIG>`中，并配置`<ROBOT>/<CONFIG>/config.yaml`和`<ROBOT>/base.yaml`中的参数。

### 仿真

#### Gazebo

打开一个终端，启动gazebo仿真环境

```bash
# ROS1
source devel/setup.bash
roslaunch rl_sar gazebo.launch rname:=<ROBOT>

# ROS2
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT>
```

打开一个新终端，启动控制程序

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_sim

# ROS2
source install/setup.bash
ros2 run rl_sar rl_sim
```

> [!TIP]
> Ubuntu22.04中若启动Gazebo后看不到机器人，则是机器人初始化到了视野范围外，启动rl_sim后会自动重置机器人位置。若机器人在站立过程中翻倒，请使用键盘`R`或手柄`RB+Y`重置机器人环境。

如果第一次启动Gazebo无法打开则需要下载模型包

```bash
git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
```

#### Mujoco

```bash
./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>
# Example: ./cmake_build/bin/rl_sim_mujoco g1 scene_29dof
```

### 使用手机网页控制 (实验性)

安装依赖

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
sudo apt install ros-${ROS_DISTRO}-web-video-server

# 如果您使用的ROS2版本不是Humble、Jazz或Rolling，需要从源码编译 `web_video_server`
cd <your_ros2_workspace>/src
git clone https://github.com/RobotWebTools/web_video_server.git
cd <your_ros2_workspace>
colcon build --packages-select web_video_server
```

在机器人上运行 rosbridge 和 web_video_server

```bash
# ROS1
roslaunch rosbridge_server rosbridge_websocket.launch
rosrun web_video_server web_video_server

# ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 run web_video_server web_video_server
```

访问 [http://robot.robotsfan.com/](http://robot.robotsfan.com/)， 填写IP地址和端口，检查右上角的设置界面，然后连接机器人。进入控制页面后，将屏幕水平放置，点击左上角的全屏按钮，即可使用手机浏览器控制机器人！

### 使用手柄或键盘控制

|手柄控制|键盘控制|功能描述|
|---|---|---|
|**基础**|||
|A|Num0|让机器人从程序开始运行时的姿态以位控插值运动到`base.yaml`中定义的`default_dof_pos`|
|B|Num9|让机器人从当前位置以位控插值运动到程序开始运行时的姿态|
|X|N|切换导航模式 (导航模式屏蔽速度命令，接收`cmd_vel`话题)|
|Y|N/A|N/A|
|**仿真**|||
|RB+Y|R|重置Gazebo环境 (让摔倒的机器人站起来)|
|RB+X|Enter|切换Gazebo运行/停止 (默认为运行状态)|
|**电机**|||
|LB+A|M|N/A (推荐设置为电机使能)|
|LB+B|K|N/A (推荐设置为电机失能)|
|LB+X|P|电机Passive模式 (`kp=0, kd=8`)|
|LB+RB|N/A|N/A (推荐设置为急停保护)|
|**技能**|||
|RB+DPadUp|Num1|基础Locomotion|
|RB+DPadDown|Num2|技能2|
|RB+DPadLeft|Num3|技能3|
|RB+DPadRight|Num4|技能4|
|LB+DPadUp|Num5|技能5|
|LB+DPadDown|Num6|技能6|
|LB+DPadLeft|Num7|技能7|
|LB+DPadRight|Num8|技能8|
|**移动**|||
|LY轴|W/S|前后移动 (X轴)|
|LX轴|A/D|左右移动 (Y轴)|
|RX轴|Q/E|偏航旋转 (Yaw)|
|N/A(松开摇杆)|Space|将所有控制指令设置为零|

### 真实机器人

<details>

<summary>Unitree A1（点击展开）</summary>

与Unitree A1连接可以使用无线与有线两种方式

- 无线：连接机器人发出的Unitree开头的WIFI **（注意：无线连接可能会出现丢包断联甚至失控，请注意安全）**
- 有线：用网线连接计算机和机器人的任意网口，配置计算机地址为192.168.123.162，子网掩码255.255.255.0

新建终端，启动控制程序

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_a1

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_a1

# CMake
./cmake_build/bin/rl_real_a1
```

</details>

<details>

<summary>Unitree Go2/Go2W/G1(29dofs)（点击展开）</summary>

#### 网线连接

用网线的一端连接Go2/Go2W/G1(29dofs)机器人，另一端连接你的电脑，并开启电脑的 USB Ethernet 后进行配置。机器狗机载电脑的 IP 地地址为 `192.168.123.161`，故需将电脑 USB Ethernet 地址设置为与机器狗同一网段，如在 Address 中输入 `192.168.123.222` (`222`可以改成其他)。

通过`ifconfig`命令查看123网段的网卡名字，如`enxf8e43b808e06`，下文用 \<YOUR_NETWORK_INTERFACE\> 代替

Go2:

新建终端，启动控制程序。如果控制Go2W，需要在命令后加`wheel`，否则留空。

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# CMake
./cmake_build/bin/rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]
```

G1(29dofs):

开机后将机器人吊起来，按L2+R2进入调试模式，然后新建终端，启动控制程序。

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_g1 <YOUR_NETWORK_INTERFACE>

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_g1 <YOUR_NETWORK_INTERFACE>

# CMake
./cmake_build/bin/rl_real_g1 <YOUR_NETWORK_INTERFACE>
```

#### 在机载Jetson中部署

使用网线连接电脑和机器人，登陆Jetson主机，密码123：

```bash
ssh unitree@192.168.123.18
```

将手机连接机器人的USB，开启手机的USB网络共享，拉取代码并使用 `./build.sh -m` 编译，编译成功后运行：

```bash
# Go2:
./cmake_build/bin/rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# G1(29dofs):
./cmake_build/bin/rl_real_g1 <YOUR_NETWORK_INTERFACE>
```

然后即可拔掉手机和网线，使用遥控器控制。

#### 开机自启动

如需设置开机自启动，可以参考以下流程：

创建一个服务文件

```bash
sudo touch /etc/systemd/system/rl_sar.service
```

写入以下内容，假设rl_sar工程在 `~/rl_sar` 目录下

```
[Unit]
Description=RL SAR Service
After=network.target

[Service]
Type=simple
User=unitree
WorkingDirectory=/home/unitree/rl_sar
ExecStart=/home/unitree/rl_sar/cmake_build/bin/rl_real_go2 eth0 wheel
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

重新加载 systemd 配置：

```bash
sudo systemctl daemon-reload
```

设置开机自动启动：

```bash
sudo systemctl enable rl_sar.service
```

禁用开机自动启动：

```bash
sudo systemctl disable rl_sar.service
```

启动服务：

```bash
sudo systemctl enable rl_sar.service
```

停止服务：

```bash
sudo systemctl stop rl_sar.service
```

重启服务：

```bash
sudo systemctl restart rl_sar.service
```

查看服务日志：

```bash
sudo journalctl -u rl_sar.service -f
```

重启后机机器人会先运行内置站立程序，rl_sar服务启动后则会自动阻尼趴下，随后可以使用遥控器正常控制。

</details>

<details>

<summary>云深处科技 Lite3 (Click to expand)</summary>

Lite3通过无线网络进行连接。
(由于一些型号的Lite3没有开放网线接口，需要额外安装，所以有线连接方式暂时没有进行测试)

- 连接Lite3的Wifi，并测试通信状况。我们强烈建议在运行本项目之前，先通过 [Lite3_Motion_SDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)进行测试和检查，在确认一切正常后再运行。
 **(注意：无线连接可能会出现丢包断联甚至失控，请注意安全)**

- 确认所使用Lite3的IP地址和本地端口与目标端口号码，并设置 **在 rl_sar/src/rl_real_lite3.cpp的行46-48**中.
- 在Lite3的运动主机中设置 **jy_exe/conf/network.toml**，使其IP地址指向与Lite3同一网段的本机，建立基于UDP的双向通信.

> [!CAUTION]
> **检查关节映射参数<br>检查确认 rl_sar/policy/himloco/config.yaml中的joint mappng参数。在Sim2Sim中使用的默认joint mapping参数与实机部署时的joint mapping是不同的，如果使用错误可能造成机器人错误的行为，带来潜在的硬件损坏和安全风险。**

Lite3也支持使用云深处Retroid手柄控制，详情参见[Deeprobotics Gamepad](https://github.com/DeepRoboticsLab/gamepad)

新建终端，启动控制程序

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_lite3

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_lite3

# CMake
./cmake_build/bin/rl_real_lite3
```

</details>

### 训练执行器网络

下面拿A1举例

1. 取消注释`rl_real_a1.hpp`中最上面的`#define CSV_LOGGER`，你也可以在仿真程序中修改对应部分采集仿真数据用来测试训练过程。
2. 运行控制程序，程序会记录所有数据到`src/rl_sar/policy/<ROBOT>/motor.csv`。
3. 停止控制程序，开始训练执行器网络。注意，下面的路径前均省略了`rl_sar/src/rl_sar/policy/`。
    ```bash
    rosrun rl_sar actuator_net.py --mode train --data a1/motor.csv --output a1/motor.pt
    ```
4. 验证已经训练好的训练执行器网络。
    ```bash
    rosrun rl_sar actuator_net.py --mode play --data a1/motor.csv --output a1/motor.pt
    ```

## 添加你的机器人

下面使用 **\<ROBOT\>/\<CONFIG\>** 代替表示你的机器人环境。你只需要创建或修改下述文件，命名必须跟下面一样。（你可以参考go2w对应的文件）

```yaml
# 你的机器人description
rl_sar/src/rl_sar_zoo/<ROBOT>_description/CMakeLists.txt
rl_sar/src/rl_sar_zoo/<ROBOT>_description/package.ros1.xml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/package.ros2.xml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/xacro/robot.xacro
rl_sar/src/rl_sar_zoo/<ROBOT>_description/xacro/gazebo.xacro
rl_sar/src/rl_sar_zoo/<ROBOT>_description/config/robot_control.yaml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/config/robot_control_ros2.yaml

# 你训练的policy
policy/<ROBOT>/base.yaml  # 此文件中必须遵守实物机器人的关节顺序
policy/<ROBOT>/<CONFIG>/config.yaml
policy/<ROBOT>/<CONFIG>/<POLICY>.pt  # libtorch使用，注意导出jit
policy/<ROBOT>/<CONFIG>/<POLICY>.onnx  # onnxruntime使用

# 机器人的fsm
src/rl_sar/fsm_robot/fsm_<ROBOT>.hpp
src/rl_sar/fsm_robot/fsm_all.hpp

# 你实物机器人的代码
rl_sar/src/rl_sar/src/rl_real_<ROBOT>.cpp  # 可以按需自定义forward()函数以适配您的policy
```

## 贡献

衷心欢迎社区的贡献，以使这个框架更加成熟和对所有人有用。贡献可以是bug报告、功能请求或代码贡献。

[贡献者名单](CONTRIBUTORS.md)

## 引用

如果您使用此代码或其部分内容，请引用以下内容：

```
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```

## 致谢

本项目使用了以下开源代码库中的部分代码：

- [unitreerobotics/unitree_sdk2-2.0.0](https://github.com/unitreerobotics/unitree_sdk2/tree/2.0.0)
- [unitreerobotics/unitree_legged_sdk-v3.2](https://github.com/unitreerobotics/unitree_legged_sdk/tree/v3.2)
- [unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)
- [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
- [google-deepmind/mujoco-3.2.7](https://github.com/google-deepmind/mujoco)
- [mertgungor/unitree_model_control](https://github.com/mertgungor/unitree_model_control)
- [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways)
- [ccrpRepo/RoboMimic_Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy)
- [Deeprobotics/Lite3_Motion_SDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)
- [chengyangkj/ROS_Flutter_Gui_App](https://github.com/chengyangkj/ROS_Flutter_Gui_App)
