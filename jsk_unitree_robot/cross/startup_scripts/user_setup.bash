#!/bin/bash

source /opt/jsk/System/system_setup.bash
source /opt/jsk/User/devel/setup.bash

export ROS_IP=$(ifconfig -a|grep inet|grep -v 127.0.0.1|grep -v inet6| head -n 1 | awk '{print $2}'|tr -d "addr:")
export ROS_MASTER_URI=http://192.168.123.161:11311

echo "ROS_IP=$ROS_IP"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
