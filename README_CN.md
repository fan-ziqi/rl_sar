# rl_sar

[English document](README.md)

强化学习的Gazebo仿真验证与UnitreeA1实物部署。"sar"代表"simulation and real"

## 准备

拉取代码（同步拉取子模块）

```bash
git clone --recursive https://github.com/fan-ziqi/rl_sar.git
```

如有更新：

```bash
git pull
git submodule update --remote --recursive
```

在任意位置下载并部署`libtorch`

```bash
cd /path/to/your/torchlib
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=/path/to/your/torchlib' >> ~/.bashrc
```

安装 `teleop-twist-keyboard` 

```bash
sudo apt install ros-noetic-teleop-twist-keyboard
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

运行前请将训练好的pt模型文件拷贝到`rl_sar/src/rl_sar/models`中

### 仿真

新建终端，启动gazebo仿真环境

```bash
source devel/setup.bash
roslaunch rl_sar start_env.launch
```

新建终端，启动控制程序

```bash
source devel/setup.bash
rosrun rl_sar rl_sim
```

新建终端，键盘控制程序

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 实物

与实物的连接可分为无线与有线形式

* 无线：连接机器人发出的Unitree开头的WIFI **（注意：无线连接可能会出现丢包断联甚至失控，请注意安全）**
* 有线：用网线连接计算机和机器人的任意网口，配置计算机ip为192.168.123.162，网关255.255.255.0

新建终端，启动控制程序

```bash
source devel/setup.bash
rosrun rl_sar rl_real
```

按下遥控器的**R2**键让机器人切换到默认站起姿态，按下**R1**键切换到RL控制模式，任意状态按下**L2**切换到最初的趴下姿态。左摇杆上下控制x左右控制yaw，右摇杆左右控制y。
