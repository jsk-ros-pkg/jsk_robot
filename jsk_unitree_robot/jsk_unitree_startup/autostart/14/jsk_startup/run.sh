#!/bin/bash
##############
# Author: ian 
##############
sleep 2

export ROS_IP=192.168.123.14
export ROS_MASTER_URI=http://192.168.123.161:11311

cd /home/unitree/jsk_catkin_ws/; source devel_isolated/setup.bash; roslaunch jsk_unitree_startup unitree_bringup.launch
RosServerID=$(ps -ef | grep node_ros_server | grep -v "grep" | wc -l)
if [ $RosServerID -ge 1 ]; then
	echo "SUCCESS"
else
	echo "node_ros_server is error, please restart the application !"
fi
