#!/bin/bash
##############
# Author: kei
# Put AFTER sportMode/imageai in ~/Unitree/autostart/.startlist.sh
##############
sleep 2

source /opt/jsk/User/user_setup.bash

eval echo "[jsk_startup] starting... " $toStartlog

# wait for roscore....
while ! eval rosnode list 2$toStartlog; do sleep 2; done
eval rosnode list $toStartlog

if [ "$ROS_IP" == "192.168.123.161" ];then
    roslaunch --screen sound_play soundplay_node.launch sound_play:=robotsound &
    roslaunch --screen jsk_unitree_startup rwt_app_chooser.launch &
fi

if [ "$ROS_IP" == "192.168.123.14" ];then
    # wait for soundplay
    while ! eval rostopic info /robotsound 2$toStartlog; do sleep 2; done
    sleep 2 # wait for a while...
    roslaunch jsk_unitree_startup unitree_bringup.launch network:=ethernet &
fi

eval echo "[jsk_startup] done... " $toStartlog
eval rosnode list $toStartlog
