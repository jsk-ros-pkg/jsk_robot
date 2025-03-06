#!/usr/bin/env bash

cp $(rospack find jsk_robot_startup)/scripts/update_workspace_main.sh /tmp/update_workspace.sh
# jsk_footstep_planner is not released for melodic
# jsk_footstep_controller is not released for melodic
# librealsense2 should not be installed from ROS repository
# realsense-ros should not be installed from ROS repository
/tmp/update_workspace.sh -r $(rospack find jsk_fetch_startup)/../jsk_fetch.rosinstall.$ROS_DISTRO -t fetch -s "jsk_footstep_controller jsk_footstep_planner librealsense2 realsense2_camera realsense2_description"
rm /tmp/update_workspace.sh
