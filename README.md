# rl_sar

[中文文档](README_CN.md)

This repository provides a framework for simulation verification and physical deployment of robot reinforcement learning algorithms, suitable for quadruped robots, wheeled robots, and humanoid robots. "sar" stands for "simulation and real"

feature:
- Support legged_gym based on IaacGym and IsaacLab based on IsaacSim. Use `framework` to distinguish.
- The code supports both cpp and python, you can find python version in `src/rl_sar/scripts`

[Click to discuss on Discord](https://discord.gg/vmVjkhVugU)

## Preparation

Clone the code

```bash
git clone https://github.com/fan-ziqi/rl_sar.git
```

## Dependency

This project uses `ros-noetic` (Ubuntu 20.04) and requires the installation of the following ROS dependency packages:

```bash
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
```

Download and deploy `libtorch` at any location

```bash
cd /path/to/your/torchlib
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=/path/to/your/torchlib' >> ~/.bashrc
```

Install `yaml-cpp` and `lcm`. If you are using Ubuntu, you can directly use the package manager for installation:

```bash
sudo apt install liblcm-dev libyaml-cpp-dev
```

<details>

<summary>You can also use source code installation, click to expand</summary>

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
</details>

## Compilation

Compile in the root directory of the project

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

#### Unitree A1

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

#### Unitree Go2

1. Connect one end of the Ethernet cable to the Go2 robot and the other end to the user's computer. Then, enable USB Ethernet on the computer and configure it. The IP address of the onboard computer on the Go2 robot is 192.168.123.161, so the computer's USB Ethernet address should be set to the same network segment as the robot. For example, enter 192.168.123.222 in the "Address" field ("222" can be replaced with another number).
2. Use the `ifconfig` command to find the name of the network interface for the 123 network segment, such as `enxf8e43b808e06`. In the following steps, replace `<YOUR_NETWORK_INTERFACE>` with the actual network interface name.
3. Open a new terminal and start the control program:
    ```bash
    source devel/setup.bash
    rosrun rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE>
    ```
4. Go2 supports both joy and keyboard control, using the same method as mentioned above for A1.

### Train the actuator network

Take A1 as an example below

1. Uncomment `#define CSV_LOGGER` in the top of `rl_real_a1.cpp`. You can also modify the corresponding part in the simulation program to collect simulation data for testing the training process.
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
4. Modify the `forward()` function in the code as needed to adapt to different models.
5. If you need to run simulations, modify the launch files as needed by referring to those in the `rl_sar/src/rl_sar/launch` directory.
6. If you need to run on the physical robot, modify the file `rl_sar/src/rl_sar/src/rl_real_a1.cpp` as needed.

## Contributing

Wholeheartedly welcome contributions from the community to make this framework mature and useful for everyone. These may happen as bug reports, feature requests, or code contributions.

[List of contributors](CONTRIBUTORS.md)

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
