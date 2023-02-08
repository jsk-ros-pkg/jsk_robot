#!/usr/bin/env bash

set -x
. /tmp/ros_run_id
. /var/lib/robot/config.bash
echo "ROS_IP: $ROS_IP";
echo "ROS_MASTER_URI: $ROS_MASTER_URI";
echo "ROS_RUN_ID: $ROS_RUN_ID";
echo "ROS_ENV_LOADER: $ROS_ENV_LOADER";
$ROS_ENV_LOADER roslaunch jsk_pr2_startup pr2.launch --screen --wait map_frame:=$MAP_FRAME
set +x
