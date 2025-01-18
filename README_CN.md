# rl_sar

[English document](README.md)

**版本选择: [ROS-Noetic](https://github.com/fan-ziqi/rl_sar/tree/main) | [ROS2-Foxy/Humble](https://github.com/fan-ziqi/rl_sar/tree/ros2)**

本仓库提供了机器人强化学习算法的仿真验证与实物部署框架，适配四足机器人、轮足机器人、人形机器人。"sar"代表"simulation and real"

特性：
- 支持基于IsaacGym的legged_gym，也支持基于IsaacSim的IsaacLab，用`framework`加以区分。
- 代码有**ROS-Noetic**和**ROS2-Foxy/Humble**两个版本
- 代码有python和cpp两个版本，python版本可以在`src/rl_sar/scripts`中找到

> [!NOTE]
> 如果你想使用IsaacLab（IsaacSim）训练策略，请使用[robot_lab](https://github.com/fan-ziqi/robot_lab)项目。
>
> [点击在Discord上讨论](https://discord.gg/MC9KguQHtt)

## 准备

拉取代码

```bash
git clone -b ros2 https://github.com/fan-ziqi/rl_sar.git
```

## 依赖

本项目使用`ros2-foxy`(Ubuntu20.04)或`ros2-humble`(Ubuntu22.04)，且需要安装以下的ros依赖包

```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros2-control ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-xacro
```

在任意位置下载并部署`libtorch`

```bash
cd /path/to/your/libtorch
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=/path/to/your/libtorch' >> ~/.bashrc
source ~/.bashrc
```

安装`yaml-cpp`和`lcm`，若您使用Ubuntu，可以直接使用包管理器进行安装

```bash
sudo apt install liblcm-dev libyaml-cpp-dev
```

<details>

<summary>也可以使用源码安装，点击展开</summary>

安装yaml-cpp

```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp && mkdir build && cd build
cmake -DYAML_BUILD_SHARED_LIBS=on .. && make
sudo make install
sudo ldconfig
```

安装lcm

```bash
git clone https://github.com/lcm-proj/lcm.git
cd lcm && mkdir build && cd build
cmake .. && make
sudo make install
sudo ldconfig
```
</details>

## 编译

在项目根目录编译

```bash
cd ..
colcon build --merge-install --symlink-install
```

## 运行

下文中使用 **\<ROBOT\>** 和 **\<PLATFORM\>** 代替表示不同的机器人和框架。目前支持列表：

|       | isaacgym | isaacsim |
|-------|----------|----------|
| a1    | ✓        | ✓        |
| go2   | ✓        |          |

运行前请将训练好的pt模型文件拷贝到`rl_sar/src/rl_sar/models/<ROBOT>_<PLATFORM>`中，并配置`config.yaml`中的参数。

### 仿真

打开一个终端，启动gazebo仿真环境

```bash
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT> framework:=<PLATFORM>
(e.g. ros2 launch rl_sar gazebo.launch.py rname:=a1 framework:=isaacgym)
```

打开一个新终端，启动控制程序

```bash
source install/setup.bash
(for cpp version)    ros2 run rl_sar rl_sim
(for python version) ros2 run rl_sar rl_sim.py
```

控制：

<!-- * 按 **\<Enter\>** 切换仿真器运行/停止。 -->
* **W** 和 **S** 控制x轴，**A** 和 **D** 控制yaw轴，**J** 和 **L** 控制y轴。
* 按 **\<Space\>** 将所有控制指令设置为零。
<!-- * 如果机器人摔倒，按 **R** 重置Gazebo环境。 -->

### 真实机器人

#### Unitree A1

与Unitree A1连接可以使用无线与有线两种方式

* 无线：连接机器人发出的Unitree开头的WIFI **（注意：无线连接可能会出现丢包断联甚至失控，请注意安全）**
* 有线：用网线连接计算机和机器人的任意网口，配置计算机ip为192.168.123.162，网关255.255.255.0

新建终端，启动控制程序

```bash
source install/local.bash
ros2 run rl_sar rl_real_a1
```

按下遥控器的**R2**键让机器人切换到默认站起姿态，按下**R1**键切换到RL控制模式，任意状态按下**L2**切换到最初的趴下姿态。左摇杆上下控制x左右控制yaw，右摇杆左右控制y。

或者按下键盘上的**0**键让机器人切换到默认站起姿态，按下**P**键切换到RL控制模式，任意状态按下**1**键切换到最初的趴下姿态。WS控制x，AD控制yaw，JL控制y。

#### Unitree Go2

1. 用网线的一端连接Go2机器人，另一端连接用户电脑，并开启电脑的 USB Ethernet 后进行配置。机器狗机载电脑的 IP 地地址为 192.168.123.161，故需将电脑 USB Ethernet 地址设置为与机器狗同一网段，如在 Address 中输入 192.168.123.222 (“222”可以改成其他)。
2. 通过`ifconfig`命令查看123网段的网卡名字，如`enxf8e43b808e06`，下文用 \<YOUR_NETWORK_INTERFACE\> 代替
3. 新建终端，启动控制程序
    ```bash
    source install/local.bash
    ros2 run rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE>
    ```
4. Go2支持手柄与键盘控制，方法与上面a1相同

### 训练执行器网络

下面拿A1举例

1. 取消注释`rl_real_a1.cpp`中最上面的`#define CSV_LOGGER`，你也可以在仿真程序中修改对应部分采集仿真数据用来测试训练过程。
2. 运行控制程序，程序会在执行后记录所有数据。
3. 停止控制程序，开始训练执行器网络。
    ```bash
    ros2 run rl_sar actuator_net.py --mode train --data a1_isaacgym/motor.csv --output a1_isaacgym/motor.pt
    ```
4. 验证已经训练好的训练执行器网络。
    ```bash
    ros2 run rl_sar actuator_net.py --mode play --data a1_isaacgym/motor.csv --output a1_isaacgym/motor.pt
    ```

## 添加你的机器人

下文中使用 **\<ROBOT\>_\<PLATFORM\>** 代替表示你的机器人环境

1. 在`rl_sar/src/robots`路径下创建名为`<ROBOT>_description`的模型包，将模型的urdf放到`rl_sar/src/robots/<ROBOT>_description/urdf`路径下并命名为`<ROBOT>.urdf`，并在`rl_sar/src/robots/<ROBOT>_description/config`路径下创建命名空间为`<ROBOT>_gazebo`的关节配置文件
2. 将训练好的RL模型文件放到`rl_sar/src/rl_sar/models/<ROBOT>_<PLATFORM>`路径下，并在此路径中新建config.yaml文件，参考`rl_sar/src/rl_sar/models/a1_isaacgym/config.yaml`文件修改其中参数
3. 按需修改代码中的`forward()`函数，以适配不同的模型
<!-- 4. 若需要运行仿真，则参考`rl_sar/src/rl_sar/launch`路径下的launch文件自行修改 -->
4. 若需要运行实物，则参考`rl_sar/src/rl_sar/src/rl_real_a1.cpp`文件自行修改

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

- [unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)
- [mertgungor/unitree_model_control](https://github.com/mertgungor/unitree_model_control)
- [src/rl_sar/scripts/actuator_net.py](src/rl_sar/scripts/actuator_net.py) 中的代码修改自 [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways) 仓库中的 [scripts/actuator_net](https://github.com/Improbable-AI/walk-these-ways/tree/master/scripts/actuator_net)

