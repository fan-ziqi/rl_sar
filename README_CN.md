# rl_sar

[![Ubuntu 20.04/22.04](https://img.shields.io/badge/Ubuntu-20.04/22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![ROS Noetic](https://img.shields.io/badge/ros-noetic-brightgreen.svg?logo=ros)](https://wiki.ros.org/noetic)
[![ROS2 Foxy/Humble](https://img.shields.io/badge/ros2-foxy/humble-brightgreen.svg?logo=ros)](https://wiki.ros.org/foxy)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2-0)

[English document](README.md)

**版本选择: [ROS-Noetic](https://github.com/fan-ziqi/rl_sar/tree/main) | [ROS2-Foxy/Humble](https://github.com/fan-ziqi/rl_sar/tree/ros2)**

> [!IMPORTANT]
> 本仓库持续维护 ROS1 的 C++ 版本，而 ROS2 版本和 Python 版本未能同步更新，部分功能可能缺失或存在差异。因此，建议优先使用 ROS1 的 C++ 版本，以获得最新的功能特性和 bug 修复。

本仓库提供了机器人强化学习算法的仿真验证与实物部署框架，适配四足机器人、轮足机器人、人形机器人。"sar"代表"simulation and real"

特性：
- 内置多种机器人仿真的预训练模型，包括 `Unitree-A1`、`Unitree-Go2`、`Unitree-Go2W`、`Unitree-B2`、`Unitree-B2W`、`FFTAI-GR1T1`、`FFTAI-GR1T2`、`GoldenRetriever-L4W0`、`GoldenRetriever-L4W4`；
- 训练框架支持**IsaacGym**和**IsaacSim**，用`framework`加以区分；
- 代码有**ROS-Noetic**和**ROS2-Foxy/Humble**两个版本；
- 代码有**python**和**cpp**两个版本，其中python版本在`src/rl_sar/scripts`内；

> [!NOTE]
> 如果你想使用IsaacLab（IsaacSim）训练策略，请使用 [robot_lab](https://github.com/fan-ziqi/robot_lab) 项目。
>
> robot_lab配置文件中的关节顺序 `joint_names` 与本项目代码中 `xxx/robot_lab/config.yaml` 中定义的相同。
>
> 在 [Github Discussion](https://github.com/fan-ziqi/rl_sar/discussions) 或 [Discord](https://discord.gg/MC9KguQHtt) 中讨论

## 准备

拉取代码

```bash
git clone https://github.com/fan-ziqi/rl_sar.git
```

## 依赖

本项目使用`ros-noetic`(Ubuntu20.04)，且需要安装以下的ros依赖包

```bash
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-joy
```

在任意位置下载并部署`libtorch`（请修改下面的 **\<YOUR_PATH\>** 为实际路径）

```bash
cd <YOUR_PATH>/libtorch
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=<YOUR_PATH>/libtorch' >> ~/.bashrc
source ~/.bashrc
```

安装`yaml-cpp`和`lcm`，若您使用Ubuntu，可以直接使用包管理器进行安装

```bash
sudo apt install liblcm-dev libyaml-cpp-dev
```

本项目使用Intel TBB（Threading Building Blocks）库进行线程间数据交换，若您使用Ubuntu，可直接使用包管理器进行安装

```bash
sudo apt install libtbb-dev
```

<details>

<summary>也可使用源码安装（点击展开）</summary>

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
catkin build
```

> [!NOTE]
> 如果 catkin build 报错: `Unable to find either executable 'empy' or Python module 'em'`, 在`catkin build` 之前执行 `catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3`

## 运行

下文中使用 **\<ROBOT\>/\<CONFIG\>** 代替表示不同的环境，如 `a1/isaacgym` 、 `go2/himloco`。

运行前请将训练好的pt模型文件拷贝到`rl_sar/src/rl_sar/models/<ROBOT>/<CONFIG>`中，并配置`<ROBOT>/<CONFIG>/config.yaml`和`<ROBOT>/base.yaml`中的参数。

### 仿真

打开一个终端，启动gazebo仿真环境

```bash
source devel/setup.bash
roslaunch rl_sar gazebo_<ROBOT>.launch cfg:=<CONFIG>
```

打开一个新终端，启动控制程序

```bash
source devel/setup.bash
(for cpp version)    rosrun rl_sar rl_sim
(for python version) rosrun rl_sar rl_sim.py
```

键盘控制：

- 按 **\<Enter\>** 切换仿真器运行/停止。
- 按 **0** 让机器人从仿真开始的姿态以位控插值运动到yaml中定义的`default_dof_pos`。
- 按 **p** 切换到强化学习模式。
- **W/S** 控制前后移动，**J/L** 控制左右移动，**A/D** 控制转向，按 **\<Space\>** 将所有控制指令设置为零。
- 按 **n** 切换到导航模式，屏蔽手柄命令，接收`cmd_vel`话题。
- 如果机器人摔倒，按 **R** 重置Gazebo环境。
- 按 **1** 让机器人从当前位置以位控插值运动到仿真开始的姿态。

手柄控制：

- 按 **LB** 切换仿真器运行/停止。
- 按 **RB+Y** 让机器人从仿真开始的姿态以位控插值运动到yaml中定义的`default_dof_pos`。
- 按 **RB+B** 切换到强化学习模式。
- **LY** 控制前后移动，**LX** 控制左右移动，**RX** 控制转向。
- 按 **左面的下键** 切换到导航模式，屏蔽手柄命令，接收`cmd_vel`话题。
- 如果机器人摔倒，按 **RB+X** 重置Gazebo环境。
- 按 **RB+A** 让机器人从当前位置以位控插值运动到仿真开始的姿态。

### 真实机器人

<details>

<summary>Unitree A1（点击展开）</summary>

与Unitree A1连接可以使用无线与有线两种方式

- 无线：连接机器人发出的Unitree开头的WIFI **（注意：无线连接可能会出现丢包断联甚至失控，请注意安全）**
- 有线：用网线连接计算机和机器人的任意网口，配置计算机地址为192.168.123.162，子网掩码255.255.255.0

新建终端，启动控制程序

```bash
source devel/setup.bash
rosrun rl_sar rl_real_a1
```

按下遥控器的**R2**键让机器人切换到默认站起姿态，按下**R1**键切换到RL控制模式，任意状态按下**L2**切换到最初的趴下姿态。左摇杆上下控制x，左摇杆左右控制yaw，右摇杆左右控制y。

或者按下键盘上的**0**键让机器人切换到默认站起姿态，按下**P**键切换到RL控制模式，任意状态按下**1**键切换到最初的趴下姿态。WS控制x，AD控制yaw，JL控制y。

</details>

<details>

<summary>Unitree Go2/Go2W（点击展开）</summary>

1. 用网线的一端连接Go2/Go2W机器人，另一端连接用户电脑，并开启电脑的 USB Ethernet 后进行配置。机器狗机载电脑的 IP 地地址为 192.168.123.161，故需将电脑 USB Ethernet 地址设置为与机器狗同一网段，如在 Address 中输入 192.168.123.222 (“222”可以改成其他)。
2. 通过`ifconfig`命令查看123网段的网卡名字，如`enxf8e43b808e06`，下文用 \<YOUR_NETWORK_INTERFACE\> 代替
3. 新建终端，启动控制程序。如果控制Go2W，需要在命令后加`wheel`，否则留空。
    ```bash
    source devel/setup.bash
    rosrun rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]
    ```
4. Go2/Go2W支持手柄与键盘控制，方法与上面a1相同

</details>

### 训练执行器网络

下面拿A1举例

1. 取消注释`rl_real_a1.cpp`中最上面的`#define CSV_LOGGER`，你也可以在仿真程序中修改对应部分采集仿真数据用来测试训练过程。
2. 运行控制程序，程序会在执行后记录所有数据。
3. 停止控制程序，开始训练执行器网络。注意，下面的路径前均省略了`rl_sar/src/rl_sar/models/`。
    ```bash
    rosrun rl_sar actuator_net.py --mode train --data a1/motor.csv --output a1/motor.pt
    ```
4. 验证已经训练好的训练执行器网络。
    ```bash
    rosrun rl_sar actuator_net.py --mode play --data a1/motor.csv --output a1/motor.pt
    ```

## 添加你的机器人

下文中使用 **\<ROBOT\>/\<CONFIG\>** 代替表示你的机器人环境

1. 在`rl_sar/src/robots`路径下创建名为`<ROBOT>_description`的模型包，将模型的urdf放到`rl_sar/src/robots/<ROBOT>_description/urdf`路径下并命名为`<ROBOT>.urdf`，并在`rl_sar/src/robots/<ROBOT>_description/config`路径下创建命名空间为`<ROBOT>_gazebo`的关节配置文件。
2. 将训练好的RL模型文件放到`rl_sar/src/rl_sar/models/<ROBOT>/<CONFIG>`路径下，在此路径中新建config.yaml文件，参考`rl_sar/src/rl_sar/models/a1/isaacgym/config.yaml`文件修改其中参数；在其上级目录新建base.yaml文件，参考`rl_sar/src/rl_sar/models/a1/base.yaml`文件修改其中参数。
3. 按需修改代码中的`forward()`函数，以适配不同的模型。
4. 若需要运行仿真，则参考`rl_sar/src/rl_sar/launch`路径下的launch文件自行修改。
5. 若需要运行实物，则参考`rl_sar/src/rl_sar/src/rl_real_a1.cpp`文件自行修改。

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

