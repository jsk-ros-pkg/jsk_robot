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
    roslaunch --screen jsk_unitree_startup rosserial_node.launch &
    roslaunch --screen respeaker_ros sample_respeaker.launch language:=ja-JP publish_tf:=false launch_soundplay:=false &
fi

if [ "$ROS_IP" == "192.168.123.14" ];then
    # 192.168.123.14 is force updated within install.sh for Go1 Air
    if [ "$ROS_IP" == "192.168.123.13" ];then
       # At this moment, nano1 have only 2G memory and fail to start live_human_pose.py....
       DISPLAY=:0.0 gnome-terminal -- bash -c "export GST_PLUGIN_PATH=/home/unitree/Unitree/autostart/imageai/mLComSystemFrame/ThirdParty/webSinkPipe/build; cd /home/unitree/Unitree/autostart/imageai/mLComSystemFrame/pyScripts; PYTHONPATH= python3 live_human_pose.py; exec bash"
    fi
    # Go1 Pro runs rosserial_node on nano2
    if [ "$ROS_IP" == "192.168.123.14" ];then
        roslaunch --screen jsk_unitree_startup rosserial_node.launch &
    fi
    # wait for soundplay
    while ! eval rostopic info /robotsound 2$toStartlog; do sleep 2; done
    sleep 2 # wait for a while...
    roslaunch jsk_unitree_startup unitree_bringup.launch network:=ethernet &
fi

eval echo "[jsk_startup] done... " $toStartlog
eval rosnode list $toStartlog
