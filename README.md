# rl_sar

[![Ubuntu 20.04/22.04](https://img.shields.io/badge/Ubuntu-20.04/22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![ROS Noetic](https://img.shields.io/badge/ros-noetic-brightgreen.svg?logo=ros)](https://wiki.ros.org/noetic)
[![ROS2 Foxy/Humble](https://img.shields.io/badge/ros2-foxy/humble-brightgreen.svg?logo=ros)](https://wiki.ros.org/foxy)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2-0)

[中文文档](README_CN.md)

**Version Select: [ROS-Noetic](https://github.com/fan-ziqi/rl_sar/tree/main) | [ROS2-Foxy/Humble](https://github.com/fan-ziqi/rl_sar/tree/ros2)**

This repository provides a framework for simulation verification and physical deployment of robot reinforcement learning algorithms, suitable for quadruped robots, wheeled robots, and humanoid robots. "sar" stands for "simulation and real"

feature:
- Built-in pre-trained models for multiple robot simulations, including `Unitree-A1`, `Unitree-Go2`, `Unitree-Go2W`, `Unitree-B2`, `Unitree-B2W`, `FFTAI-GR1T1`, `FFTAI-GR1T2`, `GoldenRetriever-L4W0`, `GoldenRetriever-L4W4`;
- The training framework supports **IsaacGym** and **IsaacSim**, distinguished by `framework`;
- The code has **ROS-Noetic** and **ROS2-Foxy/Humble** versions;
- The code has **cpp** and **python** versions, with the python version located in `src/rl_sar/scripts`;

> [!NOTE]
> If you want to train policy using IsaacLab(IsaacSim), please use [robot_lab](https://github.com/fan-ziqi/robot_lab) project.
>
> The order of joints in robot_lab cfg file `joint_names` is the same as that defined in `xxx/robot_lab/config.yaml` in this project.
>
> Discuss in [Github Discussion](https://github.com/fan-ziqi/rl_sar/discussions) or [Discord](https://discord.gg/vmVjkhVugU).

## Preparation

Clone the code

```bash
git clone https://github.com/fan-ziqi/rl_sar.git
```

## Dependency

This project uses `ros-noetic` (Ubuntu 20.04) and requires the installation of the following ROS dependency packages:

```bash
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-joy
```

Download and deploy `libtorch` at any location (Please modify **\<YOUR_PATH\>** below to the actual path)

```bash
cd <YOUR_PATH>/libtorch
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=<YOUR_PATH>/libtorch' >> ~/.bashrc
source ~/.bashrc
```

Install `yaml-cpp` and `lcm`. If you are using Ubuntu, you can directly use the package manager for installation:

```bash
sudo apt install liblcm-dev libyaml-cpp-dev
```

This project uses the Intel TBB (Threading Building Blocks) library to implement data exchange between different threads. If you use Ubuntu, you can directly use the package manager to install it

```bash
sudo apt install libtbb-dev
```

<details>

<summary>You can also use source code installation (Click to expand)</summary>

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
> [!NOTE]
> If catkin build report errors: `Unable to find either executable 'empy' or Python module 'em'`, run `catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3` before `catkin build`

## Running

In the following text, **\<ROBOT\>/\<CONFIG\>** is used to represent different environments, such as `a1/isaacgym` and `go2/himloco`.

Before running, copy the trained pt model file to `rl_sar/src/rl_sar/models/<ROBOT>/<CONFIG>`, and configure the parameters in `config.yaml`.

### Simulation

Open a terminal, launch the gazebo simulation environment

```bash
source devel/setup.bash
roslaunch rl_sar gazebo_<ROBOT>.launch cfg:=<CONFIG>
```

Open a new terminal, launch the control program

```bash
source devel/setup.bash
(for cpp version)    rosrun rl_sar rl_sim
(for python version) rosrun rl_sar rl_sim.py
```

Keyboard Controls

* Press **\<Enter\>** to toggle the simulator between running and stopped.
* Use **W/S** to control forward/backward movement, **A/D** to control turning, and **J/L** to control lateral movement. Press **\<Space\>** to reset all control commands to zero.
* If the robot falls, press **R** to reset the Gazebo environment.
* Press **0** to move the robot from its simulation start posture to `init_pos`, and press **1** to move the robot from `init_pos` back to its simulation start posture.

Gamepad Controls

* Press **LB** to toggle the simulator between running and stopped.
* **LY** controls forward/backward movement, **LX** controls lateral movement, and **RX** controls turning.
* If the robot falls, press **RB+X** to reset the Gazebo environment.
* Press **RB+Y** to move the robot from its simulation start posture to `init_pos`, and press **RB+A** to move the robot from `init_pos` back to its simulation start posture.

### Real Robots

<details>

<summary>Unitree A1 (Click to expand)</summary>

Unitree A1 can be connected using both wireless and wired methods:

* Wireless: Connect to the Unitree starting with WIFI broadcasted by the robot **(Note: Wireless connection may lead to packet loss, disconnection, or even loss of control, please ensure safety)**
* Wired: Use an Ethernet cable to connect any port on the computer and the robot, configure the computer IP as 192.168.123.162, and the netmask as 255.255.255.0

Open a new terminal and start the control program

```bash
source devel/setup.bash
rosrun rl_sar rl_real_a1
```

Press the **R2** button on the controller to switch the robot to the default standing position, press **R1** to switch to RL control mode, and press **L2** in any state to switch to the initial lying position. The left stick controls x-axis up and down, controls yaw left and right, and the right stick controls y-axis left and right.

Or press **0** on the keyboard to switch the robot to the default standing position, press **P** to switch to RL control mode, and press **1** in any state to switch to the initial lying position. WS controls x-axis, AD controls yaw, and JL controls y-axis.

</details>

<details>

<summary>Unitree Go2 (Click to expand)</summary>

1. Connect one end of the Ethernet cable to the Go2 robot and the other end to the user's computer. Then, enable USB Ethernet on the computer and configure it. The IP address of the onboard computer on the Go2 robot is 192.168.123.161, so the computer's USB Ethernet address should be set to the same network segment as the robot. For example, enter 192.168.123.222 in the "Address" field ("222" can be replaced with another number).
2. Use the `ifconfig` command to find the name of the network interface for the 123 network segment, such as `enxf8e43b808e06`. In the following steps, replace `<YOUR_NETWORK_INTERFACE>` with the actual network interface name.
3. Open a new terminal and start the control program:
    ```bash
    source devel/setup.bash
    rosrun rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE>
    ```
4. Go2 supports both joy and keyboard control, using the same method as mentioned above for A1.

</details>

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

In the following text, **\<ROBOT\>/\<CONFIG\>** is used to represent your robot environment.

1. Create a model package named `<ROBOT>_description` in the `rl_sar/src/robots` directory. Place the robot's URDF file in the `rl_sar/src/robots/<ROBOT>_description/urdf` directory and name it `<ROBOT>.urdf`. Additionally, create a joint configuration file with the namespace `<ROBOT>_gazebo` in the `rl_sar/src/robots/<ROBOT>_description/config` directory.
2. Place the trained RL model files in the `rl_sar/src/rl_sar/models/<ROBOT>/<CONFIG>` directory, and create a new `config.yaml` file in this path. Refer to the `rl_sar/src/rl_sar/models/a1/isaacgym/config.yaml` file to modify the parameters.
3. Modify the `forward()` function in the code as needed to adapt to different models.
4. If you need to run simulations, modify the launch files as needed by referring to those in the `rl_sar/src/rl_sar/launch` directory.
5. If you need to run on the physical robot, modify the file `rl_sar/src/rl_sar/src/rl_real_a1.cpp` as needed.

## Contributing

Wholeheartedly welcome contributions from the community to make this framework mature and useful for everyone. These may happen as bug reports, feature requests, or code contributions.

[List of contributors](CONTRIBUTORS.md)

## Citation

Please cite the following if you use this code or parts of it:

```
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```

## Acknowledgements

The project uses some code from the following open-source code repositories:

- [unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)
- [mertgungor/unitree_model_control](https://github.com/mertgungor/unitree_model_control)
- The code in [src/rl_sar/scripts/actuator_net.py](src/rl_sar/scripts/actuator_net.py) is modified from [scripts/actuator_net](https://github.com/Improbable-AI/walk-these-ways/tree/master/scripts/actuator_net) in the [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways) repository.
