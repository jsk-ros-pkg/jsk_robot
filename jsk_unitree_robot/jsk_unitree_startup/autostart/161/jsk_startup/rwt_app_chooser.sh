#!/bin/sh
##############
# Author: ian 
##############

sleep 5
export ROS_IP=192.168.123.161
export ROS_MASTER_URI=http://192.168.123.161:11311/
cd /home/pi/jsk_catkin_ws/;
source devel_isolated/setup.bash;
roslaunch --screen jsk_unitree_startup rwt_app_chooser.launch

