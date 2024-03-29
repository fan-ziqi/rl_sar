CYBERDOG MOTOR SDK
---
This SDK provides the interface of joint motors and IMU sensor mounted on CyberDog and gives users more freedom to develop their own controller. It is compatible with CyberDog firmware version 1.0.0.94 or higher. For more details, please refer to the example code Example_MotorCtrl.cpp. To deploy on real robots, please follow the following steps.

### Preparatory work
#### Dependency
Install LCM
```
$ git clone https://github.com/lcm-proj/lcm.git
$ cd lcm
$ mkdir build && cd build
$ cmake .. && make
$ sudo make install
```
Install docker

Follow the steps attached to the link: https://docs.docker.com/engine/install/ubuntu/
```
$ sudo groupadd docker # Set root permissions for docker
$ sudo usermod -aG docker $USER
```
Download the docker image required for cross compilation
```
$ wget https://cdn.cnbj2m.fds.api.mi-img.com/os-temp/loco/loco_arm64_20220118.tar
$ docker load --input loco_arm64_20220118.tar
$ docker images
```
#### Connect robot
Connect the local PC to the USB Download type-C interface of cyberdog (in the middle), and wait for the "L4T-README" pop-up window to appear
```
$ ping 192.168.55.100 # local PC assigned IP
$ ssh mi@192.168.55.1 # Login NX application board, password 123
mi@lubuntu:~$ athena_version -v # check current version >= 1.0.0.94
$ ssh root@192.168.55.233 # Log in to the motion control board
```
#### Enter motor control mode
Modify the configuration switch to activate the user control mode:
```
$ ssh root@192.168.55.233 # Log in to the motion control board
root@TinaLinux:~# cd /robot
root@TinaLinux:/robot# ./initialize.sh # copy the factory code to the r/w Development Zone (/mnt/UDISK/robot-software), switch to the developer mode and execute it only once
root@TinaLinux:/robot# vi /mnt/UDISK/robot-software/config/user_code_ctrl_mode.txt # switch mode: 1 (0: default mode, 1 user code controls motor mode), and restart the robot to take effect
```
### Compilation and deployment
#### 1. Deploy on user PC
It is difficult to ensure real-time LCM communication when running on the user PC (Linux), so only compilation verification and simple position control tests are recommended.
```
$ ping 192.168.55.233 # connect cyberdog's download interface through type C cable and make sure the communication is okay
$ ifconfig | grep -B 1 192.168.55.100 | grep "flags" | cut -d ':' -f1 # obtain the network device corresponding to the IP, generally usb0
$ sudo ifconfig usb0 multicast # replace usb0 with the 168.55.100 network device obtained above and set to multicast
$ sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev usb0 # add a route table and replace usb0 accordingly.
$ mkdir build && cd build
$ cmake ..
$ make -j4
$ ./Example_MotorCtrl
```
Note: if the LCM communication is unsuccessful, the motor control mode cannot be activated normally. Log prompt: motor control mode has not been activated successfully.
#### 2. Deploy on NX application board
Due to non real-time system, only compilation verification and simple position control test are recommended.
```
$ scp -r {sdk_path}/cyberdog_motor_sdk mi@192.168.55.1:/home/mi/ # copy the SDK source code into the application board, password 123
$ ssh mi@192.168.55.1 # Login application board
mi@lubuntu:~$ cd /home/mi/cyberdog_motor_sdk
mi@lubuntu:~$ mkdir build && cd build
mi@lubuntu:~$ cmake ..
mi@lubuntu:~$ make -j2
mi@lubuntu:~$ ping 192.168.55.233 # test communication with motion control board
mi@lubuntu:~$ ./Example_MotorCtrl
```
#### 3. Cross compilation and deploy on motion control board
In order to make the compiled file run directly on the robot, it needs to be compiled in the docker image environment where the cross compilation tool chain is deployed. The specific steps are as follows:
```
$ docker run -it --rm --name cyberdog_motor_sdk -v /home/xxx/{sdk_path}:/work/build_farm/workspace/cyberdog cr.d.xiaomi.net/athena/athena_cheetah_arm64:2.0 /bin/bash
[root:/work] # cd /work/build_farm/workspace/cyberdog/ # enter the code warehouse of docker system
[root:/work/build_farm/workspace/cyberdog] # mkdir onboard-build && cd onboard-build
[root:/work/build_farm/workspace/cyberdog] # cmake -DCMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake ..
[root:/work/build_farm/workspace/cyberdog] # make -j4 # specify cross compile tool chain and compile
[root:/work/build_farm/workspace/cyberdog] # exit
```
After successful compilation, copy the generated library and executable files libcyber_dog_sdk.so and Example_MotorCtrl to the directory /mnt/UDISK of motion control board.
```
$ cd ~/{sdk_path}/onboard-build
$ ssh root@192.168.55.233 "mkdir /mnt/UDISK/cyberdog_motor_sdk" # create a folder in the motion control board
$ scp libcyber_dog_motor_sdk.so  Example_MotorCtrl root@192.168.55.233:/mnt/UDISK/cyberdog_motor_sdk
$ ssh root@192.168.55.233
root@TinaLinux:~# cd /mnt/UDISK/cyberdog_motor_sdk
root@TinaLinux:~# export LD_LIBRARY_PATH=/mnt/UDISK/cyberdog_motor_sdk # setting so library path variable
root@TinaLinux:~# ./Example_MotorCtrl
```
To run in the background so that the process will not be interrupted by the exit of SSH connection, please use the command below instead:
```
nohup ./Example_MotorCtrl &
```

How to add boot auto start:

Configure the JSON process management file /mnt/UDISK/manager_config/fork_para_conf_lists.json, and restart the robot.

Example: "600003": {"fork_config":{"name": "Example_MotorCtrl",  "object_path": "/cyberdog_motor_sdk/",  "log_path": "", "paraValues": ["", "", ""] }}

Note: when closing the program manually, please close the user program such as Example_MotorCtrl first which will trigger the timeout protection of main program and lie down the robot, and then close or restart the main program. If the main program and user program are closed at the same time, the motor will be locked due to the timeout of CAN bus and invite dangers during restart.
#### Explanation of error flags
```
//bit0: warning flag, lost communication between user code and robot over 10[ms]. For safety, commanded tau and qd_des will be forced to divide by (over_time[ms]/10.0);
//bit1: error flag, lost communication between user code and robot over 500[ms]. Robot will enter high-damping mode by setting joint gains kp=0, kd=10, tau=0;
//bit2: warning flag, position command of any abaduction joint changing more than 8 degrees from its previous will be truncated;
//bit3: warning flag, position command of any hip joint changing more than 10 degrees from its previous will be truncated;
//bit4: warning flag, position command of any knee joint changing more than 12 degrees from its previous will be truncated;
```
Note: in order to avoid danger caused by communication timeout, when "err_flag: 0x02 communicate lost over 500ms" is reported, please identify the problem and close Example_MotorCtrl first. The error flag cannot be cleared until the main control program is restared or the control board is totally rebooted.

How to restart the main control program:
```
$ ssh root@192.168.55.233 "ps | grep -E 'Example_MotorCtrl' | grep -v grep | awk '{print \$1}' | xargs kill -9" # Example_MotorCtrl needs to stop earlier than the main control process to avoid emergency stop
$ ssh root@192.168.55.233 "ps | grep -E 'manager|ctrl|imu_online' | grep -v grep | awk '{print \$1}' | xargs kill -9"
$ ssh root@192.168.55.233 "export LD_LIBRARY_PATH=/mnt/UDISK/robot-software/build;/mnt/UDISK/manager /mnt/UDISK/ >> /mnt/UDISK/manager_log/manager.log 2>&1 &"
```
How to reboot the motion control board:
```
$ ssh root@192.168.55.233 "reboot"
```
