#!/bin/bash

WORKSPACE_SPOT_DRIVER=/home/spot/spot_driver_ws
WORKSPACE_SPOT_CORAL=/home/spot/spot_coral_ws
WORKSPACE_SPOT=/home/spot/spot_ws

if [ -e $WORKSPACE_SPOT_DRIVER ]; then
    cd $WORKSPACE_SPOT_DRIVER/src
    wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_driver.rosinstall -y -r
    wstool update -t . -j 4 -m 60 --delete-changed-uris --continue-on-error
    catkin build -j4 --continue-on-failure
else
    echo "Workspace $WORKSPACE_SPOT_DRIVER not found."
fi

if [ -e $WORKSPACE_SPOT_CORAL ]; then
    cd $WORKSPACE_SPOT_CORAL/src
    wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_coral.rosinstall -y -r
    wstool update -t . -j 4 -m 60 --delete-changed-uris --continue-on-error
    catkin build -j4 --continue-on-failure
else
    echo "Workspace $WORKSPACE_SPOT_CORAL not found."
fi

if [ -e $WORKSPACE_SPOT ]; then
    cd $WORKSPACE_SPOT/src
    wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_user.rosinstall -y -r
    wstool update -t . -j 4 -m 60 --delete-changed-uris --continue-on-error
    catkin build -j4 --continue-on-failure
else
    echo "Workspace $WORKSPACE_SPOT not found."
fi
