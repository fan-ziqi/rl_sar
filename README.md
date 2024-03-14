# rl_sar

[中文文档](README.md)

Gazebo simulation verification and UnitreeA1 physical deployment for reinforcement learning. "sar" stands for "simulation and real".

## Preparation

Clone the code (sync submodules)

```bash
git clone --recursive https://github.com/fan-ziqi/rl_sar.git
```

If there are updates:

```bash
git pull
git submodule update --remote --recursive
```

Download and deploy `libtorch` at any location

```bash
cd /path/to/your/torchlib
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=/path/to/your/torchlib' >> ~/.bashrc
```

Install `teleop-twist-keyboard`

```bash
sudo apt install ros-noetic-teleop-twist-keyboard
```

## Compilation

Customize the following two functions in your code to adapt to different models:

```cpp
torch::Tensor forward() override;
torch::Tensor compute_observation() override;
```

Then compile in the root directory

```bash
cd ..
catkin build
```

## Running

Before running, copy the trained pt model file to `rl_sar/src/unitree_rl/models`

### Simulation

Open a new terminal, launch the gazebo simulation environment

```bash
source devel/setup.bash
roslaunch unitree_rl start_env.launch
```

Open a new terminal, run the control program

```bash
source devel/setup.bash
rosrun unitree_rl unitree_rl
```

Open a new terminal, run the keyboard control program

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Physical Deployment

Open a new terminal, run the control program

```bash
source devel/setup.bash
rosrun unitree_rl unitree_rl_real
```

> Some code references: https://github.com/mertgungor/unitree_model_control