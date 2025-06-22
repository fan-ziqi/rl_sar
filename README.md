# rl_sar

[![Ubuntu 20.04/22.04/24.04](https://img.shields.io/badge/Ubuntu-20.04/22.04/24.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![ROS Noetic](https://img.shields.io/badge/ros-noetic-brightgreen.svg?logo=ros)](https://wiki.ros.org/noetic)
[![ROS2 Foxy/Humble](https://img.shields.io/badge/ros2-foxy/humble-brightgreen.svg?logo=ros)](https://wiki.ros.org/foxy)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2-0)

[中文文档](README_CN.md)

This repository provides a framework for simulation verification and physical deployment of robot reinforcement learning algorithms, suitable for quadruped robots, wheeled robots, and humanoid robots. "sar" stands for "simulation and real"

> Supports both **IsaacGym** and **IsaacSim**(distinguished by `framework`)
>
> Supports both **ROS-Noetic** and **ROS2-Foxy/Humble/Jazzy**

Support List：

| Robot Name   | Policy (IsaacGym) | Policy (IsaacSim) | ROS1 Support | ROS2 Support |
|------------|----------------------|----------------------|----------|----------|
| a1         | legged_gym            | ⚪                    | ✅       | ✅       |
| go2        | himloco               | robot_lab            | ✅       | ✅       |
| go2w       | ⚪                    | robot_lab            | ✅       | ✅       |
| b2         | ⚪                    | robot_lab            | ✅       | ✅       |
| b2w        | ⚪                    | robot_lab            | ✅       | ✅       |
| g1         | unitree_ros           | ⚪                    | ✅       | ✅       |
| gr1t1      | legged_gym            | ⚪                    | ✅       | ❌       |
| gr2t2      | legged_gym            | ⚪                    | ✅       | ❌       |
| l4w4       | legged_gym            | robot_lab            | ✅       | ✅       |

> [!IMPORTANT]
> Python version temporarily suspended maintenance, please use [v2.3](https://github.com/fan-ziqi/rl_sar/releases/tag/v2.3) if necessary, may be re-released in the future.

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
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-controller-manager
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

Since this project supports multiple versions of ROS, some symbolic links need to be created for different versions. A build script is provided in the project root directory for one-click compilation.

Execute the following script in the project root directory to compile the entire project:

```bash
./build.sh
```

To compile specific packages individually, you can append the package names:

```bash
./build.sh package1 package2
```

To clean the build, use the following command. This will remove all compiled outputs and created symbolic links:

```bash
./build.sh -c  # or ./build.sh --clean
```

If simulation is not needed and you only want to run on the robot, you can compile using CMake while disabling ROS (the compiled executables will be in `cmake_build/bin` and libraries in `cmake_build/lib`):

```bash
./build.sh -m  # or ./build.sh --cmake
```

For detailed usage instructions, you can check them via `./build.sh -h`:

```bash
Usage: ./build.sh [OPTIONS] [PACKAGE_NAMES...]

Options:
  -c, --clean    Clean workspace (remove symlinks and build artifacts)
  -m, --cmake    Build using CMake (for hardware deployment only)
  -h, --help     Show this help message

Examples:
  ./build.sh                    # Build all ROS packages
  ./build.sh package1 package2  # Build specific ROS packages
  ./build.sh -c                 # Clean all symlinks and build artifacts
  ./build.sh --clean package1   # Clean specific package and build artifacts
  ./build.sh -m                 # Build with CMake for hardware deployment
```

> [!NOTE]
> If catkin build report errors: `Unable to find either executable 'empy' or Python module 'em'`, run `catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3` before `catkin build`

## Running

In the following text, **\<ROBOT\>/\<CONFIG\>** is used to represent different environments, such as `go2/himloco` and `go2w/robot_lab`.

Before running, copy the trained pt model file to `rl_sar/src/rl_sar/policy/<ROBOT>/<CONFIG>`, and configure the parameters in `<ROBOT>/<CONFIG>/config.yaml` and `<ROBOT>/base.yaml`.

### Simulation

> [!NOTE]
> For ROS1 simulation, the joint order is already defined in `rl_sar/src/rl_sar/policy/<ROBOT>/<CONFIG>/config.yaml` and does not require manual modification.
> For ROS2 simulation, before running the simulation, you need to manually copy the joint order defined in `rl_sar/src/rl_sar/policy/<ROBOT>/<CONFIG>/config.yaml` to `rl_sar/src/robots/<ROBOT>_description/config/robot_control_group.yaml`.

Open a terminal, launch the gazebo simulation environment

```bash
# ROS1
source devel/setup.bash
roslaunch rl_sar gazebo_<ROBOT>.launch cfg:=<CONFIG>

# ROS2
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT> cfg:=<CONFIG>
```

Open a new terminal, launch the control program

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_sim

# ROS2
source install/setup.bash
ros2 run rl_sar rl_sim
```

Keyboard Control:

- Press **<Enter>** to toggle the simulator between running and stopped.
- Press **0** to move the robot from the initial simulation pose to the `default_dof_pos` defined in the YAML file using position control interpolation.
- Press **p** to switch to Reinforcement Learning mode.
- Use **W/S** to move forward/backward, **J/L** to move left/right, and **A/D** to turn. Press **<Space>** to reset all control commands to zero.
- Press **n** to switch to Navigation mode, disabling gamepad commands and receiving commands from the `cmd_vel` topic.
- If the robot falls down, press **R** to reset the Gazebo environment.
- Press **1** to move the robot from its current position back to the initial simulation pose using position control interpolation.

Gamepad Control:

- Press **LB** to toggle the simulator between running and stopped.
- Press **RB + Y** to move the robot from the initial simulation pose to the `default_dof_pos` defined in the YAML file using position control interpolation.
- Press **RB + B** to switch to Reinforcement Learning mode.
- Use **LY** to move forward/backward, **LX** to move left/right, and **RX** to turn.
- Press the **down button on the left** to switch to Navigation mode, disabling gamepad commands and receiving commands from the `cmd_vel` topic.
- If the robot falls down, press **RB + X** to reset the Gazebo environment.
- Press **RB + A** to move the robot from its current position back to the initial simulation pose using position control interpolation.

### Real Robots

<details>

<summary>Unitree A1 (Click to expand)</summary>

Unitree A1 can be connected using both wireless and wired methods:

- Wireless: Connect to the Unitree starting with WIFI broadcasted by the robot **(Note: Wireless connection may lead to packet loss, disconnection, or even loss of control, please ensure safety)**
- Wired: Use an Ethernet cable to connect any port on the computer and the robot, configure the computer IP as 192.168.123.162, and the netmask as 255.255.255.0

Open a new terminal and start the control program

```bash
source devel/setup.bash
rosrun rl_sar rl_real_a1
```

Press **R2** on the gamepad to switch the robot to the default standing pose, **R1** to switch to Reinforcement Learning (RL) control mode, and **L2** in any state to return to the initial lying-down pose. The **left joystick up/down** controls movement along the **x-axis**, **left joystick left/right** controls **yaw**, and **right joystick left/right** controls movement along the **y-axis**.

Alternatively, press **0** on the keyboard to switch the robot to the default standing pose, **P** to switch to RL control mode, and **1** in any state to return to the initial lying-down pose. Use **W/S** to control the **x-axis**, **A/D** to control **yaw**, and **J/L** to control the **y-axis**.

</details>

<details>

<summary>Unitree Go2/Go2W (Click to expand)</summary>

#### Ethernet Connection

Connect one end of the Ethernet cable to the Go2/Go2W robot and the other end to your computer. Then, enable USB Ethernet on the computer and configure it. The IP address of the onboard computer on the Go2 robot is `192.168.123.161`, so the computer's USB Ethernet address should be set to the same network segment as the robot. For example, enter `192.168.123.222` in the "Address" field (you can replace `222` with another number).

Use the `ifconfig` command to find the name of the network interface for the 123 network segment, such as `enxf8e43b808e06`. In the following steps, replace `<YOUR_NETWORK_INTERFACE>` with the actual network interface name.

Open a new terminal and start the control program. If you are controlling Go2W, you need to add `wheel` after the command, otherwise leave it blank.

```bash
source devel/setup.bash
rosrun rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]
```

Go2/Go2W supports both joy and keyboard control, following the same method as described for A1.

#### Deploying on the Onboard Jetson

Connect your computer to the robot using the Ethernet cable and log into the Jetson onboard computer. The default password is `123`:

```bash
ssh unitree@192.168.123.18
```

Check the JetPack version:

```bash
sudo pip install jetson-stats
sudo jtop
```

Download and install the PyTorch version that matches your JetPack:

```bash
wget https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
sudo apt install python-is-python3 python3.9-dev
pip install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
```

Check the installed torch path:

```bash
python -c "import torch; print(torch.__file__)"
```

Manually create the `libtorch` directory and copy necessary files

```bash
mkdir ~/libtorch
cp -r /home/unitree/.local/lib/python3.8/site-packages/torch/{bin,include,lib,share} ~/libtorch
echo 'export Torch_DIR=~/libtorch' >> ~/.bashrc
source ~/.bashrc
```

Clone the source code and compile it as described in the previous sections.

</details>

### Train the actuator network

Take A1 as an example below

1. Uncomment `#define CSV_LOGGER` in the top of `rl_real_a1.hpp`. You can also modify the corresponding part in the simulation program to collect simulation data for testing the training process.
2. Run the control program, and the program will log all data after execution.
3. Stop the control program and start training the actuator network. Note that `rl_sar/src/rl_sar/policy/` is omitted before the following paths.
    ```bash
    rosrun rl_sar actuator_net.py --mode train --data a1/motor.csv --output a1/motor.pt
    ```
4. Verify the trained actuator network.
    ```bash
    rosrun rl_sar actuator_net.py --mode play --data a1/motor.csv --output a1/motor.pt
    ```

## Add Your Robot

In the following context, use **\<ROBOT\>/\<CONFIG\>** to represent your robot environment:

1. Under the path `rl_sar/src/robots`, create a model package named `<ROBOT>_description`, and create joint configuration files under the path `rl_sar/src/robots/<ROBOT>_description/config/`. Please refer to existing files for specific details.
2. Place the trained RL policy files under the path `rl_sar/src/rl_sar/policy/<ROBOT>/<CONFIG>/`. Create a config.yaml file in this path and create a base.yaml file in its parent directory. Please refer to existing files for specific details.
3. Modify the `forward()` function in the code as needed to adapt to different policies.
4. If you need to run simulation, refer to the launch files under the path `rl_sar/src/rl_sar/launch/` and modify accordingly.
5. If you need to run on physical hardware, refer to the file `rl_sar/src/rl_sar/src/rl_real_go2.cpp` and modify accordingly.

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
