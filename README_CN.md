# rl_sar

[English document](README.md)

机器人强化学习算法的仿真验证与实物部署，适配四足机器人、轮足机器人、人形机器人。"sar"代表"simulation and real"

本框架支持基于IaacGym的legged_gym，也支持基于IsaacSim的IsaacLab，使用时用`framework`加以区分。

[点击在Discord上讨论](https://discord.gg/MC9KguQHtt)

## 准备

拉取代码

```bash
git clone https://github.com/fan-ziqi/rl_sar.git
```

## 依赖

本项目依赖ROS-Noetic(Ubuntu20.04)

安装好ros之后安装依赖库

```bash
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
```

在任意位置下载并部署`libtorch`

```bash
cd /path/to/your/torchlib
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=/path/to/your/torchlib' >> ~/.bashrc
```

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

## 编译

自定义代码中的以下两个函数，以适配不同的模型：

```cpp
torch::Tensor forward() override;
torch::Tensor compute_observation() override;
```

然后到根目录编译

```bash
cd ..
catkin build
```

## 运行

运行前请将训练好的pt模型文件拷贝到`rl_sar/src/rl_sar/models/YOUR_ROBOT_NAME`中，并配置`config.yaml`中的参数。

### 仿真

打开一个终端，启动gazebo仿真环境

```bash
source devel/setup.bash
roslaunch rl_sar gazebo_<ROBOT>.launch
```

打开一个新终端，启动控制程序

```bash
source devel/setup.bash
(for cpp version)    rosrun rl_sar rl_sim
(for python version) rosrun rl_sar rl_sim.py
```

其中 \<ROBOT\> 可以是 `a1` 或 `a1_isaaclab` 或 `gr1t1` 或 `gr1t2`.

控制：

* 按 **\<Enter\>** 切换仿真器运行/停止。
* **W** 和 **S** 控制x轴，**A** 和 **D** 控制yaw轴，**J** 和 **L** 控制y轴，按下空格重置控制指令。
* 按 **\<Space\>** 将所有控制指令设置为零。
* 如果机器人摔倒，按 **R** 重置Gazebo环境。

### 实物

#### Unitree A1

与Unitree A1连接可以使用无线与有线两种方式

* 无线：连接机器人发出的Unitree开头的WIFI **（注意：无线连接可能会出现丢包断联甚至失控，请注意安全）**
* 有线：用网线连接计算机和机器人的任意网口，配置计算机ip为192.168.123.162，网关255.255.255.0

新建终端，启动控制程序

```bash
source devel/setup.bash
rosrun rl_sar rl_real_a1
```

按下遥控器的**R2**键让机器人切换到默认站起姿态，按下**R1**键切换到RL控制模式，任意状态按下**L2**切换到最初的趴下姿态。左摇杆上下控制x左右控制yaw，右摇杆左右控制y。

OR 按下键盘上的**0**键让机器人切换到默认站起姿态，按下**P**键切换到RL控制模式，任意状态按下**1**键切换到最初的趴下姿态。WS控制x，AD控制yaw，JL控制y。

## 添加你的机器人

下文中将ROBOT代表机器人名称

1. 在robots文件夹中创建名为ROBOT_description的模型包，将模型的urdf放到文件夹中的urdf路径下并命名为ROBOT.urdf，在模型文件中的config文件夹中创建命名空间为ROBOT_gazebo的关节配置文件
2. 将模型文件放到models/ROBOT中
3. 在rl_sar/config.yaml中添加一个新的字段，命名为ROBOT，更改其中参数，如将model_name改为上一步的模型文件名
4. 在rl_sar/launch文件夹中添加一个新的launch文件，请参考其他launch文件自行修改
5. 修改rl_xxx.cpp中的ROBOT_NAME为ROBOT
6. 编译运行

## 参考

[unitree_ros](https://github.com/unitreerobotics/unitree_ros)

## 引用

如果您使用此代码或其部分内容，请引用以下内容：

```
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {{rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.}},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```