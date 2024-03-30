#!/bin/bash
set -e
set -u

# 获取IP地址对应的网络设备
network_device=$(ifconfig | grep -B 1 192.168.55.100 | grep "flags" | cut -d ':' -f1)

# 配置网络设备为多播
sudo ifconfig $network_device multicast

# 添加路由表
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev $network_device