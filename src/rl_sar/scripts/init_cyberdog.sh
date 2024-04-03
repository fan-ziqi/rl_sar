#!/bin/bash
set +e
set -u

HOMENAME=$(basename $HOME)

network_device=$(ifconfig | grep -B 1 192.168.55.100 | grep "flags" | cut -d ':' -f1)
sudo ifconfig $network_device multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev $network_device

sudo sed -i 's/#\s*StrictHostKeyChecking/StrictHostKeyChecking/' /etc/ssh/ssh_config
if grep -q "StrictHostKeyChecking" /etc/ssh/ssh_config; then
    sudo sed -i 's/StrictHostKeyChecking.*/StrictHostKeyChecking no/' /etc/ssh/ssh_config
else
    echo "StrictHostKeyChecking no" | sudo tee -a /etc/ssh/ssh_config
fi

ssh-keygen -f "/home/${HOMENAME}/.ssh/known_hosts" -R "192.168.55.233"

bash kill_cyberdog.sh