#!/bin/bash
set -e
set -u

ssh root@192.168.55.233 "ps | grep -E 'manager|ctrl|imu_online' | grep -v grep | awk '{print \$1}' | xargs kill -9"
ssh root@192.168.55.233 "export LD_LIBRARY_PATH=/mnt/UDISK/robot-software/build;/mnt/UDISK/manager /mnt/UDISK/ >> /mnt/UDISK/manager_log/manager.log 2>&1 &"