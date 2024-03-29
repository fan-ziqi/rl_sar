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

与实物的连接可分为无线与有线形式

* 无线：连接机器人发出的Unitree开头的WIFI **（注意：无线连接可能会出现丢包断联甚至失控，请注意安全）**
* 有线：用网线连接计算机和机器人的任意网口，配置计算机ip为192.168.123.162，网关255.255.255.0

新建终端，启动控制程序

```bash
source devel/setup.bash
rosrun rl_sar rl_real
```

按下遥控器的**R2**键让机器人切换到默认站起姿态，按下**R1**键切换到RL控制模式，任意状态按下**L2**切换到最初的趴下姿态。左摇杆上下控制x左右控制yaw，右摇杆左右控制y。

#### Cyberdog1

1. 连接机器人

    将本地PC连接至铁蛋的USB download type-c 接口(位于中间)，等待出现”L4T-README” 弹窗

      ```bash
      ping 192.168.55.100     #本地PC被分配的ip
      ssh mi@192.168.55.1     #登录nx应用板 ,密码123
      athena_version -v #核对当前版本>=1.0.0.94
      ```
    
2. 进入电机控制模式

    修改配置开关，激活用户控制模式，运行用户自己的控制器：

      ```bash
      ssh root@192.168.55.233 #登录运动控制板
      cd /robot
      ./initialize.sh #拷贝出厂代码到可读写的开发区（/mnt/UDISK/robot-software），切换到开发者模式，仅需执行一次
      vi /mnt/UDISK/robot-software/config/user_code_ctrl_mode.txt #切换mode:1(0:默认模式，1用户代码控制电机模式),重启机器人生效
      ```


3. 用户电脑侧部署

    运行在用户pc侧(linux)难以保证实时lcm通信，仅推荐编译验证和简单的位控测试  

      ```bash
      ping 192.168.55.233 #通过type c线连接Cyberdog的Download接口后，确认通信正常
      ifconfig | grep -B 1 192.168.55.100 | grep "flags"| cut -d ':' -f1 #获取该ip对应网络设备，一般为usb0
      sudo ifconfig usb0 multicast #usb0替换为上文获取的168.55.100对应网络设备,并配为多播
      sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev usb0 #添加路由表，usb0对应替换
      ```
    
    启动控制程序
    
    ```bash
    source devel/setup.bash
    rosrun rl_sar rl_real
    ```
    
    按下键盘上的**0**键让机器人切换到默认站起姿态，按下**P**键切换到RL控制模式，任意状态按下**1**键切换到最初的趴下姿态。WS控制x，AD控制yaw，JL控制y。
4. 重启

    ```bash
    # 重启运控程序:
    ssh root@192.168.55.233 "ps | grep -E 'Example_MotorCtrl' | grep -v grep | awk '{print \$1}' | xargs kill -9" #需先于主进程暂停，避免急停
    ssh root@192.168.55.233 "ps | grep -E 'manager|ctrl|imu_online' | grep -v grep | awk '{print \$1}' | xargs kill -9"
    ssh root@192.168.55.233 "export LD_LIBRARY_PATH=/mnt/UDISK/robot-software/build;/mnt/UDISK/manager /mnt/UDISK/ >> /mnt/UDISK/manager_log/manager.log 2>&1 &"
    # 重启运控板系统:
    ssh root@192.168.55.233 "reboot"
    ```

    

注：lcm通信若不成功，无法正常激活电机控制模式，log提示：Motor control mode has not been activated successfully

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