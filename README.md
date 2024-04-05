# rl_sar

[中文文档](README_CN.md)

Simulation verification and physical deployment of the quadruped robot's reinforcement learning algorithm. "sar" stands for "simulation and real".

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

## Dependency

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

Install yaml-cpp

```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp && mkdir build && cd build
cmake -DYAML_BUILD_SHARED_LIBS=on .. && make
sudo make install
sudo ldconfig
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

Before running, copy the trained pt model file to `rl_sar/src/rl_sar/models/YOUR_ROBOT_NAME`, and configure the parameters in `config.yaml`.

### Simulation

Open a new terminal, launch the gazebo simulation environment

```bash
source devel/setup.bash
roslaunch rl_sar start_a1.launch
```

Open a new terminal, run the control program

```bash
source devel/setup.bash
rosrun rl_sar rl_sim
```

Open a new terminal, run the keyboard control program

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Physical Deployment

The connection to the physical robot can be done in both wireless and wired forms.

- Wireless: Connect to the WIFI starting with "Unitree" emitted by the robot **(Note: Wireless connection may experience packet loss, disconnection, or even loss of control, please pay attention to safety)**
- Wired: Use an Ethernet cable to connect any port on the computer to the robot, configure the computer's IP as 192.168.123.162, and the gateway as 255.255.255.0.

Create a new terminal and launch the control program.

```bash
source devel/setup.bash
rosrun rl_sar rl_real
```

Press the **R2** button on the remote control to switch the robot to the default standing posture, press **R1** to switch to RL control mode, and press **L2** in any state to switch back to the initial lying posture. The left joystick controls x-axis up and down, controls yaw left and right, and the right joystick controls y-axis left and right.

## Citation

Please cite the following if you use this code or parts of it:

```
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {{rl_sar: Simulation Verification and Physical Deployment of the Quadruped Robot's Reinforcement Learning Algorithm.}},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```