# Gallant Gazebo Sim2sim Implementation
This repo is derived from `RL_SAR` and for `IsaacSim` to `Gazebo` verification. For more details of installation and other kind of usage, please turn to `README.md`.
## Installation
### Setup
* Ubuntu 22.04
* ROS2 humble (recommended desktop version)
* cuda 12.04
* cudnn 9.12.0
* onnxruntime-linux-x64-gpu-1.22.0
## Compilation
```
./build.sh
```
## Usage
Remember to close all conda envs
* Terminal 1 (publish topic */desired_goal_pose*)
```
source install/setup.bash
ros2 launch rl_sar rviz.launch.py
```
* Terminal 2
```
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=g1
```
* Terminal 3
```
source install/setup.bash
ros2 run rl_sar rl_sim
0 # setup the env
1 # start to use onnx policy to control robot
```