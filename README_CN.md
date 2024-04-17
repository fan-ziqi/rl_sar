# rl_sar

[English document](README.md)

四足机器人强化学习算法的仿真验证与实物部署。"sar"代表"simulation and real"

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

## 依赖

在任意位置下载并部署`libtorch`

```bash
cd /path/to/your/torchlib
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=/path/to/your/torchlib' >> ~/.bashrc
```

安装依赖库

```bash
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
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

新建终端，启动gazebo仿真环境

```bash
source devel/setup.bash
roslaunch rl_sar start_a1.launch
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

#### Cyberdog1

1. 连接机器人(只需执行一次此步骤)

    将本地PC连接至铁蛋的USB download type-c 接口(位于中间)，等待出现”L4T-README” 弹窗

    ```bash
    ping 192.168.55.100     #本地PC被分配的ip
    ssh mi@192.168.55.1     #登录nx应用板 ,密码123
    athena_version -v #核对当前版本>=1.0.0.94
    ```
    
2. 进入电机控制模式(只需执行一次此步骤)

    修改配置开关，激活用户控制模式，运行用户自己的控制器：

    ```bash
    ssh root@192.168.55.233 #登录运动控制板
    cd /robot
    ./initialize.sh #拷贝出厂代码到可读写的开发区（/mnt/UDISK/robot-software），切换到开发者模式，仅需执行一次
    vi /mnt/UDISK/robot-software/config/user_code_ctrl_mode.txt #切换mode:1(0:默认模式，1用户代码控制电机模式),重启机器人生效
    ```

3. 使用网线连接电脑和运动控制板

    由于使用Type-C连接时调试碰撞易损坏接口，而且通信延迟较高，故推荐使用网线进行连接。需要将机器人拆开，断开断开主控和运动控制板的网线，将电脑和运动控制板使用网线直接连接，并设置电脑的有线连接IPv4为手动`192.168.55.100`。推荐拆掉头部并将网线从头部的开口引出。拆装时候注意不要损坏排线。

    初始化机器人的连接(每次重新连接机器人都要执行此步骤)

    ```bash
    cd src/rl_sar/scripts
    bash init_cyberdog.sh
    ```
    
    启动控制程序
    
    ```bash
    source devel/setup.bash
    rosrun rl_sar rl_real_cyberdog
    ```
    
    按下键盘上的**0**键让机器人切换到默认站起姿态，按下**P**键切换到RL控制模式，任意状态按下**1**键切换到最初的趴下姿态。WS控制x，AD控制yaw，JL控制y。

4. 使用Type-C线连接电脑与机器人

    若不方便拆卸机器人，可以暂时使用Type-C线调试。接入Type-C线后运行方法同上。

5. 程序在使用Ctrl+C结束后会自动重置机器人的运控程序，如程序失控也可手动重启运控程序。
   
   注：运控程序重启后大概有5-10秒的启动时间，在这段时间内运行程序会报`Motor control mode has not been activated successfully`，需等待不报错再运行控制程序。

    ```bash
    cd src/rl_sar/scripts
    bash kill_cyberdog.sh
    ```

## 引用

如果您使用此代码或其部分内容，请引用以下内容：

```
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {{rl_sar: Simulation Verification and Physical Deployment of the Quadruped Robot's Reinforcement Learning Algorithm.}},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```