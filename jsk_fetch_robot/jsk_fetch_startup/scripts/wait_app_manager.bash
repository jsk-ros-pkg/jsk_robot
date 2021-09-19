#!/usr/bin/env bash

echo "waiting for app manager to come up"
. /opt/ros/$ROS_DISTRO/setup.sh
ROBOT_NAME=`rosparam get /robot/name`
while [ "$ROBOT_NAME" = '' ]
do
    ROBOT_NAME=`rosparam get /robot/name`
    sleep 1;
done

ret=`rostopic echo /$ROBOT_NAME/app_list -n 1`
while [ "$ret" = '' ]
do
    ret=`rostopic echo /$ROBOT_NAME/app_list -n 1`
    sleep 1;
done
