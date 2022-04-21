#!/bin/sh
##############
# Author: ian 
##############

sleep 5
export ROS_IP=192.168.123.161
export ROS_MASTER_URI=http://192.168.123.161:11311/
cd /home/pi/jsk_catkin_ws/;
source devel_isolated/setup.bash;
roslaunch --screen sound_play soundplay_node.launch sound_play:=robotsound

