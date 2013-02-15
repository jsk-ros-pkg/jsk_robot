#!/bin/bash
set +e
PR2_NAME=$1
if [ "$1" == "" ]; then PR2_NAME="pr1012"; fi
shift


export PS1=pr2admin
source ~/.bashrc

export ROS_MASTER_URI=http://${PR2_NAME}:11311
## for desktop
sleep 1
rosrun pr2_dashboard pr2_dashboard &
xterm -sb -rightbar -sl 99999 +s -title "RVIZ" -e "export PS1=pr2admin; source ~/.bashrc; rossetip; rossetrobot ${PR2_NAME}; ROS_MASTER_URI=http://${PR2_NAME}:11311 rosrun rviz rviz -d $(rospack find jsk_pr2_startup)/config/jsk_startup.vcg" &

## for robot
sleep 1
ssh -t -l applications ${PR2_NAME} "export PS1=applications; source ~/.bashrc; ROS_MASTER_URI=http://${PR2_NAME}:11311 roslaunch jsk_pr2_startup pr2.launch"
