#!/bin/bash

### for normal network setting
#USE_IP=133.11.216.xxx

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

#rossetmaster ${USE_IP}
#rossetip ${USE_IP}
rossetip
rossetmaster $ROS_IP

source $(rospack find jsk_panda_startup)/scripts/dot.bash_byobu_function

byobu-create-session franka

byobu-new-window franka roscore  roscore
sleep 1

byobu-new-window franka main   roslaunch --wait jsk_panda_startup dual_panda.launch

byobu-new-window franka coral  "source ~/coral_ws/devel/setup.bash && roslaunch --wait coral_usb edgetpu_object_detector.launch INPUT_IMAGE:=/camera/rgb/image_rect_color"

byobu-new-window franka teleop  roslaunch --wait rwt_teleop franka_screenpoint_teleop.launch

byobu-new-window franka rviz     rviz -d ~/Desktop/dual_panda.rviz

byobu
