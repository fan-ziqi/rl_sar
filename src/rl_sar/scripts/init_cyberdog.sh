#!/bin/bash
set -e
set -u

ifconfig | grep -B 1 192.168.55.100 | grep "flags"| cut -d ':' -f1 #获取该ip对应网络设备，一般为usb0
sudo ifconfig usb0 multicast #usb0替换为上文获取的168.55.100对应网络设备,并配为多播
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev usb0 #添加路由表，usb0对应替换