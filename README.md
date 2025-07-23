# rl_sar

[![Ubuntu 20.04/22.04](https://img.shields.io/badge/Ubuntu-20.04/22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![ROS Noetic](https://img.shields.io/badge/ros-noetic-brightgreen.svg?logo=ros)](https://wiki.ros.org/noetic)
[![ROS2 Foxy/Humble](https://img.shields.io/badge/ros2-foxy/humble-brightgreen.svg?logo=ros)](https://wiki.ros.org/foxy)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2-0)

[中文文档](README_CN.md)

This repository provides a framework for simulation verification and physical deployment of robot reinforcement learning algorithms, suitable for quadruped robots, wheeled robots, and humanoid robots. "sar" stands for "simulation and real"

> Supports both **IsaacGym** and **IsaacSim**
>
> Supports both **ROS-Noetic** and **ROS2-Foxy/Humble**

Support List:

|Robot Name (rname:=)|Pre-Trained Policy|Real|
|-|-|-|
|Unitree-A1 (a1)|legged_gym (IsaacGym)|✅|
|Unitree-Go2 (go2)|himloco (IsaacGym)</br>robot_lab (IsaacSim)|✅</br>✅|
|Unitree-Go2W (go2w)|robot_lab (IsaacSim)|✅|
|Unitree-B2 (b2)|robot_lab (IsaacSim)|⚪|
|Unitree-B2W (b2w)|robot_lab (IsaacSim)|⚪|
|Unitree-G1 (g1)|unitree_rl_gym (IsaacGym)</br>robomimic pre-loco (IsaacGym)</br>robomimic_dance (IsaacGym)</br>robomimic_kick (IsaacGym)</br>robomimic_kungfu (IsaacGym)|✅</br>✅</br>✅</br>🚫</br>🚫|
|FFTAI-GR1T1 (gr1t1)</br>(Only available on Ubuntu20.04)|legged_gym (IsaacGym)|⚪|
|FFTAI-GR1T2 (gr1t2)</br>(Only available on Ubuntu20.04)|legged_gym (IsaacGym)|⚪|
|GoldenRetriever-L4W4 (l4w4)|legged_gym (IsaacGym)</br>robot_lab (IsaacSim)|✅</br>✅|
|Deeprobotics-Lite3 (lite3)|himloco (IsaacGym)|✅|

> [!IMPORTANT]
> Python version temporarily suspended maintenance, please use [v2.3](https://github.com/fan-ziqi/rl_sar/releases/tag/v2.3) if necessary, may be re-released in the future.

> [!NOTE]
> If you want to train policy using IsaacLab(IsaacSim), please use [robot_lab](https://github.com/fan-ziqi/robot_lab) project.
>
> The order of joints in robot_lab cfg file `joint_names` is the same as that defined in `xxx/robot_lab/config.yaml` in this project.
>
> Discuss in [Github Discussion](https://github.com/fan-ziqi/rl_sar/discussions).

> [!CAUTION]
> **Disclaimer: User acknowledges that all risks and consequences arising from using this code shall be solely borne by the user, the author assumes no liability for any direct or indirect damages, and proper safety measures must be implemented prior to operation.**

## Preparation

Clone the code

```bash
git clone https://github.com/fan-ziqi/rl_sar.git
```

## Dependency

If you are using `ros-noetic` (Ubuntu 20.04), you need to install the following ROS packages:

```bash
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-controller-manager
```

If you are using `ros2-foxy` (Ubuntu 20.04) or `ros2-humble` (Ubuntu 22.04), you need to install the following ROS2 packages:

```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros2-control ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-xacro
```

Download and deploy `libtorch` at any location (Please modify **\<YOUR_PATH\>** below to the actual path)

```bash
cd <YOUR_PATH>
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=<YOUR_PATH>/libtorch' >> ~/.bashrc
source ~/.bashrc
```

Install `yaml-cpp` and `lcm`. If you are using Ubuntu, you can directly use the package manager for installation:

```bash
sudo apt install liblcm-dev libyaml-cpp-dev
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

> [!TIP]
> If catkin build report errors: `Unable to find either executable 'empy' or Python module 'em'`, run `catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3` before `catkin build`

## Running

In the following text, **\<ROBOT\>/\<CONFIG\>** is used to represent different environments, such as `go2/himloco` and `go2w/robot_lab`.

Before running, copy the trained pt model file to `rl_sar/src/rl_sar/policy/<ROBOT>/<CONFIG>`, and configure the parameters in `<ROBOT>/<CONFIG>/config.yaml` and `<ROBOT>/base.yaml`.

### Simulation

Open a terminal, launch the gazebo simulation environment

```bash
# ROS1
source devel/setup.bash
roslaunch rl_sar gazebo.launch rname:=<ROBOT>

# ROS2
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT>
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

> [!TIP]
> If you cannot see the robot after launching Gazebo in Ubuntu 22.04, it means the robot was initialized outside the field of view. The robot's position will be automatically reset after launching rl_sim. If the robot falls over during the standing process, use the keyboard `R` or the gamepad `RB+Y` to reset the robot.

If Gazebo cannot be opened when you start it for the first time, you need to download the model package

```bash
git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
```

### Gamepad and Keyboard Controls

|Gamepad Control|Keyboard Control|Description|
|---|---|---|
|**Basic**|||
|A|Num0|Move the robot from its initial program pose to the `default_dof_pos` defined in `base.yaml` using position control interpolation|
|B|Num9|Move the robot from its current position to the initial program pose using position control interpolation|
|X|N|Toggle navigation mode (disables velocity commands, receives `cmd_vel` topic)|
|Y|N/A|N/A|
|**Simulation**|||
|RB+Y|R|Reset Gazebo environment (stand up fallen robot)|
|RB+X|Enter|Toggle Gazebo run/stop (default: running state)|
|**Motor**|||
|LB+A|M|N/A (Recommended for motor enable)|
|LB+B|K|N/A (Recommended for motor disable)|
|LB+X|P|N/A Motor passive mode (`kp=0, kd=8`)|
|LB+RB|N/A|N/A (Recommended for emergency stop)|
|**Skill**|||
|RB+DPadUp|Num1|Basic Locomotion|
|RB+DPadDown|Num2|Skill 2|
|RB+DPadLeft|Num3|Skill 3|
|RB+DPadRight|Num4|Skill 4|
|LB+DPadUp|Num5|Skill 5|
|LB+DPadDown|Num6|Skill 6|
|LB+DPadLeft|Num7|Skill 7|
|LB+DPadRight|Num8|Skill 8|
|**Movement**|||
|LY Axis|W/S|Forward/Backward movement (X-axis)|
|LX Axis|A/D|Left/Right movement (Y-axis)|
|RX Axis|Q/E|Yaw rotation|
|N/A (Release joystick)|Space|Reset all control commands to zero|

### Real Robots

<details>

<summary>Unitree A1 (Click to expand)</summary>

Unitree A1 can be connected using both wireless and wired methods:

- Wireless: Connect to the Unitree starting with WIFI broadcasted by the robot **(Note: Wireless connection may lead to packet loss, disconnection, or even loss of control, please ensure safety)**
- Wired: Use an Ethernet cable to connect any port on the computer and the robot, configure the computer IP as 192.168.123.162, and the netmask as 255.255.255.0

Open a new terminal and start the control program

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_a1

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_a1

# CMake
./cmake_build/bin/rl_real_a1
```

</details>

<details>

<summary>Unitree Go2/Go2W/G1(29dofs) (Click to expand)</summary>

#### Ethernet Connection

Connect one end of the Ethernet cable to the Go2/Go2W/G1(29dofs) robot and the other end to your computer. Then, enable USB Ethernet on the computer and configure it. The IP address of the onboard computer on the Go2 robot is `192.168.123.161`, so the computer's USB Ethernet address should be set to the same network segment as the robot. For example, enter `192.168.123.222` in the "Address" field (you can replace `222` with another number).

Use the `ifconfig` command to find the name of the network interface for the 123 network segment, such as `enxf8e43b808e06`. In the following steps, replace `<YOUR_NETWORK_INTERFACE>` with the actual network interface name.

Go2:

Open a new terminal and start the control program. If you are controlling Go2W, you need to add `wheel` after the command, otherwise leave it blank.

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# CMake
./cmake_build/bin/rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]
```

G1(29dofs):

Turn on the robot and lift it up, press L2+R2 to enter the debugging mode, then open a new terminal and start the control program.

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_g1 <YOUR_NETWORK_INTERFACE>

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_g1 <YOUR_NETWORK_INTERFACE>

# CMake
./cmake_build/bin/rl_real_g1 <YOUR_NETWORK_INTERFACE>
```

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

Pull the code and compile it. The process is the same as above.

</details>

<details>

<summary>Deeprobotics Lite3 (Click to expand)</summary>

Deeprobotics Lite3 can be connected using wireless method.
(Wired not tested. For some versions of Lite3, the wired Ethernet port may requires additional installation.)

- Connect to the Lite3 starting with WIFI broadcasted by the robot. We strongly recommand testing the communication the Lite3 using [Lite3_Motion_SDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK) before use.
 **(Note: Wireless connection may lead to packet loss, disconnection, or even loss of control, please ensure safety)**

- Determine the IP address and port number of Lite3, and modify **line 46-48 in rl_sar/src/rl_real_lite3.cpp**.
- Then Update **jy_exe/conf/network.toml** on the Lite3 motion host to set the IP and port to that of the local machine running ROS2, enabling communication.

> [!CAUTION]
> **Recheck joint mapping parameters!<br>Recheck rl_sar/policy/himloco/config.yaml. The default joint mapping in Sim2Sim configuration differs from that used in real. If not updated accordingly, this mismatch may lead to incorrect robot behavior and potential safety hazards**

Lite3 also support control using Deeprobotics Retroid gamepad, refer to [Deeprobotics Gamepad](https://github.com/DeepRoboticsLab/gamepad)

Open a new terminal and start the control program

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_lite3

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_lite3

# CMake
./cmake_build/bin/rl_real_lite3
```

</details>

### Train the actuator network

Take A1 as an example below

1. Uncomment `#define CSV_LOGGER` in the top of `rl_real_a1.hpp`. You can also modify the corresponding part in the simulation program to collect simulation data for testing the training process.
2. Run the control program, and the program will log all data in `src/rl_sar/policy/<ROBOT>/motor.csv`.
3. Stop the control program and start training the actuator network. Note that `rl_sar/src/rl_sar/policy/` is omitted before the following paths.
    ```bash
    rosrun rl_sar actuator_net.py --mode train --data a1/motor.csv --output a1/motor.pt
    ```
4. Verify the trained actuator network.
    ```bash
    rosrun rl_sar actuator_net.py --mode play --data a1/motor.csv --output a1/motor.pt
    ```

## Add Your Robot

The following uses **\<ROBOT\>/\<CONFIG\>** to represent your robot environment, with all paths relative to `rl_sar/src/`. You only need to create or modify the following files, and the names must exactly match those shown below. (You can refer to the corresponding files in go2w as examples.)

```yaml
# your robot description
robots/<ROBOT>_description/CMakeLists.txt
robots/<ROBOT>_description/package.ros1.xml
robots/<ROBOT>_description/package.ros2.xml
robots/<ROBOT>_description/xacro/robot.xacro
robots/<ROBOT>_description/xacro/gazebo.xacro
robots/<ROBOT>_description/config/robot_control.yaml
robots/<ROBOT>_description/config/robot_control_ros2.yaml

# your policy
rl_sar/policy/fsm.hpp
rl_sar/policy/<ROBOT>/fsm.hpp
rl_sar/policy/<ROBOT>/base.yaml  # This file must follow the physical robot's joint order
rl_sar/policy/<ROBOT>/<CONFIG>/config.yaml
rl_sar/policy/<ROBOT>/<CONFIG>/<POLICY>.pt  # Must be exported as JIT to be usable

# your real robot code
rl_sar/src/rl_real_<ROBOT>.cpp  # You can customize the forward() function as needed to adapt to your policy
```

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

- [unitreerobotics/unitree_sdk2-2.0.0](https://github.com/unitreerobotics/unitree_sdk2/tree/2.0.0)
- [unitreerobotics/unitree_legged_sdk-v3.2](https://github.com/unitreerobotics/unitree_legged_sdk/tree/v3.2)
- [unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)
- [mertgungor/unitree_model_control](https://github.com/mertgungor/unitree_model_control)
- [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways)
- [ccrpRepo/RoboMimic_Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy)
- [Deeprobotics/Lite3_Motion_SDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)
