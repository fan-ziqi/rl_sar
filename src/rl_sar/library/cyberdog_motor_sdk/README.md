CYBERDOG MOTOR SDK
---
此SDK开放了电机驱动器和机身IMU传感器接口，配合cyberdog 1.0.0.94及以上版本使用，方便用户进行运动控制的二次开发。具体接口使用可参照Example_MotorCtrl.cpp，按如下步骤在实际机器人上部署运行。

### 准备工作
#### 安装依赖
安装lcm(本地部署时需要)
```
$ git clone https://github.com/lcm-proj/lcm.git
$ cd lcm
$ mkdir build && cd build
$ cmake .. && make
$ sudo make install
```
安装docker(运控部署时需要)

按照链接所附步骤进行安装：https://docs.docker.com/engine/install/ubuntu/
```
$ sudo apt-get remove docker docker-engine docker.io containerd runc
$ sudo apt-get update
$ sudo apt-get install \
    ca-certificates \
    curl \
    gnupg \
    lsb-release
$ sudo mkdir -p /etc/apt/keyrings
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
$ echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
$ sudo apt-get update
$ sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin
$ sudo docker run hello-world

# 给docker设置root权限：
$ sudo groupadd docker
$ sudo usermod -aG docker $USER
$ sudo gpasswd -a $USER docker     #将登陆用户加入到docker用户组中
```
下载交叉编译所需docker镜像
```
$ wget https://cdn.cnbj2m.fds.api.mi-img.com/os-temp/loco/loco_arm64_20220118.tar
$ docker load --input loco_arm64_20220118.tar
$ docker images
```
#### 连接机器人
将本地PC连接至铁蛋的USB download type-c 接口(位于中间)，等待出现”L4T-README” 弹窗
```
$ ping 192.168.55.100     #本地PC被分配的ip
$ ssh mi@192.168.55.1     #登录nx应用板 ,密码123
mi@lubuntu:~$ athena_version -v #核对当前版本>=1.0.0.94
$ ssh root@192.168.55.233 #登录运动控制板
```
#### 进入电机控制模式
修改配置开关，激活用户控制模式，运行用户自己的控制器：
```
$ ssh root@192.168.55.233 #登录运动控制板
root@TinaLinux:~# cd /robot
root@TinaLinux:~# ./initialize.sh #拷贝出厂代码到可读写的开发区（/mnt/UDISK/robot-software），切换到开发者模式，仅需执行一次
root@TinaLinux:~# vi /mnt/UDISK/robot-software/config/user_code_ctrl_mode.txt #切换mode:1(0:默认模式，1用户代码控制电机模式),重启机器人生效
```
### 编译及部署

#### 1、用户电脑侧部署
运行在用户pc侧(linux)难以保证实时lcm通信，仅推荐编译验证和简单的位控测试  
```
$ ping 192.168.55.233 #通过type c线连接Cyberdog的Download接口后，确认通信正常
$ ifconfig | grep -B 1 192.168.55.100 | grep "flags"| cut -d ':' -f1 #获取该ip对应网络设备，一般为usb0
$ sudo ifconfig usb0 multicast #usb0替换为上文获取的168.55.100对应网络设备,并配为多播
$ sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev usb0 #添加路由表，usb0对应替换
下载sdk
$ cd cyberdog_motor_sdk/
$ mkdir build && cd build
$ cmake ..
$ make -j4
$ ./Example_MotorCtrl
```
注：lcm通信若不成功，无法正常激活电机控制模式，log提示：Motor control mode has not been activated successfully

#### 2、铁蛋NX应用板部署
因非实时系统，仅推荐编译验证和简单位控测试
```
$ scp -r {sdk_path}/cyberdog_motor_sdk mi@192.168.55.1:/home/mi/ #sdk源码拷入应用板，密码123
$ ssh mi@192.168.55.1 #登录应用板
mi@lubuntu:~$ cd /home/mi/cyberdog_motor_sdk
mi@lubuntu:~$ mkdir build && cd build
mi@lubuntu:~$ cmake ..
mi@lubuntu:~$ make -j2
mi@lubuntu:~$ ping 192.168.55.233 #测试和运控板的通信
mi@lubuntu:~$ ./Example_MotorCtrl
```
#### 3、铁蛋运控板交叉编译部署
为了能使编译的文件可以直接在机器人上运行，需要在部署交叉编译工具链的docker镜像环境下编译，具体步骤如下：

```
$ docker run -it --rm --name cyberdog_motor_sdk -v /home/xxx/{sdk_path}:/work/build_farm/workspace/cyberdog cr.d.xiaomi.net/athena/athena_cheetah_arm64:2.0 /bin/bash

docker run -it --rm --name cyberdog_motor_sdk -v /home/fzq614/ROS_Workspaces/cyberdog_motor_sdk:/work/build_farm/workspace/cyberdog cr.d.xiaomi.net/athena/athena_cheetah_arm64:2.0 /bin/bash

[root:/work] # cd /work/build_farm/workspace/cyberdog/ #进入docker系统的代码仓
[root:/work/build_farm/workspace/cyberdog] # mkdir onboard-build && cd onboard-build
[root:/work/build_farm/workspace/cyberdog] # cmake -DCMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake ..
[root:/work/build_farm/workspace/cyberdog] # make -j4 #指定交叉编译工具链并编译
[root:/work/build_farm/workspace/cyberdog] # exit
```
编译成功后, 将生成的.so文件libcyber_dog_sdk.so和可执行文件Example_MotorCtrl拷贝到运控/mnt/UDISK目录下
```
$ cd ~/{sdk_path}/onboard-build

cd ~/ROS_Workspaces/cyberdog_motor_sdk/onboard-build

$ ssh root@192.168.55.233 "mkdir /mnt/UDISK/cyberdog_motor_sdk" #在运控板内创建文件夹
$ scp libcyber_dog_motor_sdk.so  Example_MotorCtrl root@192.168.55.233:/mnt/UDISK/cyberdog_motor_sdk
$ ssh root@192.168.55.233
root@TinaLinux:~# cd /mnt/UDISK/cyberdog_motor_sdk
root@TinaLinux:~# export LD_LIBRARY_PATH=/mnt/UDISK/cyberdog_motor_sdk #设置so库路径变量
root@TinaLinux:~# ./Example_MotorCtrl  #通过“nohup ./Example_MotorCtrl &”可后台运行，退出ssh连接不受影响
```
如何添加开机自启动:  
配置/mnt/UDISK/manager_config/fork_para_conf_lists.json 进程管理文件(注意结尾逗号)后重启运控程序  
例：  "600003": {"fork_config":{"name": "Example_MotorCtrl",  "object_path": "/cyberdog_motor_sdk/",  "log_path": "", "paraValues": ["", "", ""] }}  
注：手动关闭程序时，请先关闭用户程序Example_MotorCtrl，触发主程序(ctrl)超时保护趴下，再关闭或重启主程序。同时关闭主程序和用户程序，电机会因CAN总线超时位置锁定，再次启动易发生危险。

#### 错误标志位含义
```
//bit0: warning flag, lost communication between user code and robot over 10[ms]. For safety, commanded tau and qd_des will be forced to divide by (over_time[ms]/10.0);
//bit1: error flag, lost communication between user code and robot over 500[ms]. Robot will enter high-damping mode by setting joint gains kp=0, kd=10, tau=0;
//bit2: warning flag, position command of any abaduction joint changing more than 8 degrees from its previous will be truncated;
//bit3: warning flag, position command of any hip joint changing more than 10 degrees from its previous will be truncated;
//bit4: warning flag, position command of any knee joint changing more than 12 degrees from its previous will be truncated;
```
注：为了避免通信超时导致危险，报err_flag: 0x02 communicate lost over 500ms后先排除故障,关闭Example_MotorCtrl例程进程,再重启运控程序或者直接重启运控板才能清除错误.
```
# 重启运控程序:
$ ssh root@192.168.55.233 "ps | grep -E 'Example_MotorCtrl' | grep -v grep | awk '{print \$1}' | xargs kill -9" #需先于主进程暂停，避免急停
$ ssh root@192.168.55.233 "ps | grep -E 'manager|ctrl|imu_online' | grep -v grep | awk '{print \$1}' | xargs kill -9"
$ ssh root@192.168.55.233 "export LD_LIBRARY_PATH=/mnt/UDISK/robot-software/build;/mnt/UDISK/manager /mnt/UDISK/ >> /mnt/UDISK/manager_log/manager.log 2>&1 &"
# 重启运控板系统:
$ ssh root@192.168.55.233 "reboot"
```
