# rl_sim2sim

[English document](README.md)

## 准备

拉取代码（同步拉取子模块）

```bash
git clone --recursive https://github.com/fan-ziqi/rl_sim2sim.git
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

将训练好的pt模型文件拷贝到`sim2sim/src/unitree_rl/models`中

新建终端，启动gazebo仿真环境

```bash
source devel/setup.bash
roslaunch unitree_rl start_env.launch
```

新建终端，启动控制程序

```bash
source devel/setup.bash
rosrun unitree_rl unitree_rl
```

新建终端，键盘控制程序

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```



> 部分代码参考https://github.com/mertgungor/unitree_model_control

0.003563, 0.001512, -0.101311
-0.026536, 0.062404, 0.000014
-0.000302, 0.000558, -0.011532, 0.999933
0.499042, 1.120411, -2.696528, -0.497325, 1.120405, -2.696527, 0.495533, 1.120391, -2.696531, -0.493813, 1.120387, -2.696530
0.838333, 0.031000, -0.103122, -0.768373, -0.006074, 0.000378, 0.669494, 0.020684, -0.066489, -0.588053, -0.004475, 0.000998

-0.014956, -0.002124, 0.007345
0.005116, 0.010299, -0.021976, 0.999692
0.484257, 0.969105, -2.556143, -0.369799, 1.061818, -2.635455, 0.314972, 1.066411, -2.661583, -0.353911, 1.047658, -2.649109
0.000000, 0.000000, 0.012878, 0.000000, -0.012878, 0.127060, 0.006010, 0.000000, -0.050652, 0.042926, -0.024897, -0.091003

0.146841, 1.004310, -1.390954, -0.053719, 0.855462, -1.598009, -0.011909, 0.792030, -1.664962, -0.052048, 1.163194, -1.519249
-0.062282, -0.523800, 0.810542, 0.027546, -0.772218, 1.406845, -0.005030, -0.555534, 1.106206, 0.010507, -0.411735, 0.578516