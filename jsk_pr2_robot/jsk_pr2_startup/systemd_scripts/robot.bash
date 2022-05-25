#!/usr/bin/env bash

set -x
. /tmp/ros_run_id
. /var/lib/robot/config.bash
echo "ROS_RUN_ID: $ROS_RUN_ID";
echo "ROS_ENV_LOADER: $ROS_ENV_LOADER";
robot claim -f;
robot stop -f;
sleep 3;
$ROS_ENV_LOADER roslaunch /etc/ros/robot.launch --pid /var/tmp/ros.id --run_id=$ROS_RUN_ID
set +x
