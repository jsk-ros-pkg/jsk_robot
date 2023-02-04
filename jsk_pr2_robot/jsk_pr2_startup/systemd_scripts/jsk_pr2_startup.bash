#!/usr/bin/env bash

set -x
. /tmp/ros_run_id
$ROS_ENV_LOADER roslaunch jsk_pr2_startup pr2.launch --screen --wait map_frame:=$MAP_FRAME
set +x
