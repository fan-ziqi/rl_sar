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

### Physical Robots

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

#### Cyberdog1

1. Connect to the robot (only need to do this once)

    Connect the local PC to the Cyberdog's USB download Type-C interface (located in the middle) and wait for the "L4T-README" pop-up to appear.

    ```bash
    ping 192.168.55.100     # IP assigned to the local PC
    ssh mi@192.168.55.1     # Log in to the NX application board, password 123
    athena_version -v # Verify the current version is >=1.0.0.94
    ```

2. Enter motor control mode (only need to do this once)

    Modify the configuration switch to activate user control mode and run the user's own controller:

    ```bash
    ssh root@192.168.55.233 # Log in to the motion control board
    cd /robot
    ./initialize.sh # Copy factory code to the readable and writable development area (/mnt/UDISK/robot-software), switch to developer mode, only need to be executed once
    vi /mnt/UDISK/robot-software/config/user_code_ctrl_mode.txt # Switch mode: 1 (0: default mode, 1 user code control motor mode), reboot the robot to take effect
    ```

3. Use Ethernet cable to connect the computer and the motion control board

    Due to the risk of damaging the interface and higher communication latency when using a Type-C connection, it is recommended to use an Ethernet cable for connection. Disconnect the cables between the main control and motion control board of the robot, and connect the computer and the motion control board directly with an Ethernet cable. It is recommended to remove the head and lead the cable out of the head opening. Be careful not to damage the cables during disassembly and assembly.

    Initialize the robot's connection (this step needs to be done every time the robot is reconnected)

    ```bash
    cd src/rl_sar/scripts
    bash init_cyberdog.sh
    ```

    Start the control program

    ```bash
    source devel/setup.bash
    rosrun rl_sar rl_real_cyberdog
    ```

    Press **0** on the keyboard to switch the robot to the default standing position, press **P** to switch to RL control mode, and press **1** in any state to switch to the initial lying position. WS controls x-axis, AD controls yaw, and JL controls y-axis.

4. Use a Type-C cable to connect the computer and the robot

    If it is inconvenient to disassemble the robot, a Type-C cable can be temporarily used for debugging. The procedure after connecting the Type-C cable is the same as above.

5. After using Ctrl+C to end the program, the robot's motion control program will automatically reset. If the program goes out of control, the motion control program can also be manually restarted.

    Note: After restarting the motion control program, there is a startup time of approximately 5-10 seconds. During this time, running programs may report `Motor control mode has not been activated successfully`. Wait until there are no errors before running the control program again.

    ```bash
    cd src/rl_sar/scripts
    bash kill_cyberdog.sh
    ```

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