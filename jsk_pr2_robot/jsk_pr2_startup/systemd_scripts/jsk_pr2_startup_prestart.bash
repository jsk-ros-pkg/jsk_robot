#!/usr/bin/env bash

exec >"/tmp/ros_run_id"
echo "ROS_IP=$(ip -o route get 8.8.8.8 | awk '{print $7;}')"
