#!/bin/bash

source /home/nao/System/system_setup.bash
source /home/nao/User/devel/setup.bash

export ROS_IP=$(ip addr show wlan0 | grep -Po '(?<= inet )([0-9]{1,3}.){3}[0-9]{1,3}')
export ROS_MASTER_URI=http://${ROS_IP}:11311
export NAO_IP=${ROS_IP}

echo "ROS_IP=$ROS_IP"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
