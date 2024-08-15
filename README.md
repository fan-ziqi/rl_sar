# rl_sar

[中文文档](README_CN.md)

Simulation verification and physical deployment of robot reinforcement learning algorithms, suitable for quadruped robots, wheeled robots, and humanoid robots. "sar" stands for "simulation and real"

This framework supports legged_gym based on IaacGym and IsaacLab based on IsaacSim. Use `framework` to distinguish.

[Click to discuss on Discord](https://discord.gg/vmVjkhVugU)

## Preparation

Clone the code

```bash
git clone https://github.com/fan-ziqi/rl_sar.git
```

## Dependency

This project relies on ROS Noetic (Ubuntu 20.04)

After installing ROS, install the dependency library

```bash
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
```

Download and deploy `libtorch` at any location

```bash
cd /path/to/your/torchlib
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=/path/to/your/torchlib' >> ~/.bashrc
```

Install yaml-cpp

```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp && mkdir build && cd build
cmake -DYAML_BUILD_SHARED_LIBS=on .. && make
sudo make install
sudo ldconfig
```

Install lcm

```bash
git clone https://github.com/lcm-proj/lcm.git 
cd lcm && mkdir build && cd build
cmake .. && make
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

If catkin build report errors: `Unable to find either executable 'empy' or Python module 'em'`, run `catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3` before `catkin build`

## Running

Before running, copy the trained pt model file to `rl_sar/src/rl_sar/models/YOUR_ROBOT_NAME`, and configure the parameters in `config.yaml`.

### Simulation

Open a terminal, launch the gazebo simulation environment

```bash
source devel/setup.bash
roslaunch rl_sar gazebo_<ROBOT>.launch
```

Open a new terminal, launch the control program

```bash
source devel/setup.bash
(for cpp version)    rosrun rl_sar rl_sim
(for python version) rosrun rl_sar rl_sim.py
```

Where \<ROBOT\> can be `a1` or `a1_isaaclab` or `gr1t1` or `gr1t2`.

Control:
* Press **\<Enter\>** to toggle simulation start/stop.
* **W** and **S** controls x-axis, **A** and **D** controls yaw, and **J** and **L** controls y-axis.
* Press **\<Space\>** to sets all control commands to zero.
* If robot falls down, press **R** to reset Gazebo environment.

### Real Robots

**Example: Unitree A1**

Unitree A1 can be connected using both wireless and wired methods:

* Wireless: Connect to the Unitree starting with WIFI broadcasted by the robot **(Note: Wireless connection may lead to packet loss, disconnection, or even loss of control, please ensure safety)**
* Wired: Use an Ethernet cable to connect any port on the computer and the robot, configure the computer IP as 192.168.123.162, and the gateway as 255.255.255.0

Open a new terminal and start the control program

```bash
source devel/setup.bash
rosrun rl_sar rl_real_a1
```

Press the **R2** button on the controller to switch the robot to the default standing position, press **R1** to switch to RL control mode, and press **L2** in any state to switch to the initial lying position. The left stick controls x-axis up and down, controls yaw left and right, and the right stick controls y-axis left and right.

Or press **0** on the keyboard to switch the robot to the default standing position, press **P** to switch to RL control mode, and press **1** in any state to switch to the initial lying position. WS controls x-axis, AD controls yaw, and JL controls y-axis.

### Train the actuator network

1. Uncomment `#define CSV_LOGGER` in the top of `rl_real.cpp`. You can also modify the corresponding part in the simulation program to collect simulation data for testing the training process.
2. Run the control program, and the program will log all data after execution.
3. Stop the control program and start training the actuator network. Note that `rl_sar/src/rl_sar/models/` is omitted before the following paths.
    ```bash
    rosrun rl_sar actuator_net.py --mode train --data a1/motor.csv --output a1/motor.pt
    ```
4. Verify the trained actuator network.
    ```bash
    rosrun rl_sar actuator_net.py --mode play --data a1/motor.csv --output a1/motor.pt
    ```

## Add Your Robot

In the following text, `<ROBOT>` represents the name of the robot

1. Create a model package named `<ROBOT>_description` in the `rl_sar/src/robots` directory. Place the robot's URDF file in the `rl_sar/src/robots/<ROBOT>_description/urdf` directory and name it `<ROBOT>.urdf`. Additionally, create a joint configuration file with the namespace `<ROBOT>_gazebo` in the `rl_sar/src/robots/<ROBOT>_description/config` directory.
2. Place the trained RL model files in the `rl_sar/src/rl_sar/models/<ROBOT>` directory.
3. In the `rl_sar/src/rl_sar/models/<ROBOT>` directory, create a `config.yaml` file, and modify its parameters based on the `rl_sar/src/rl_sar/models/a1_isaacgym/config.yaml` file.
4. If you need to run simulations, modify the launch files as needed by referring to those in the `rl_sar/src/rl_sar/launch` directory.
5. If you need to run on the physical robot, modify the file `rl_sar/src/rl_sar/src/rl_real_a1.cpp` as needed.

## Reference

[unitree_ros](https://github.com/unitreerobotics/unitree_ros)

## Citation

Please cite the following if you use this code or parts of it:

```
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {{rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.}},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```
