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
rosrun rl_sar unitree_rl
```

新建终端，键盘控制程序

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 实物

新建终端，启动控制程序

```bash
source devel/setup.bash
rosrun rl_sar unitree_rl_real
```

> 部分代码参考https://github.com/mertgungor/unitree_model_control
