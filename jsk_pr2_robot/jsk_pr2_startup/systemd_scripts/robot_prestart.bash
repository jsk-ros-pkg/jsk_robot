#!/usr/bin/env bash

set -x
exec >"/tmp/ros_run_id"
echo "export ROS_IP=$(ip -o route get 8.8.8.8 | awk '{print $7;}')"
echo "export ROS_RUN_ID=$(date +%Y%m%d-%H%M%S)_$(uuidgen -t)"
set +x
