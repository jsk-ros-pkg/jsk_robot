#!/bin/bash

WORKSPACE_SPOT_DRIVER=/home/spot/spot_driver_ws
WORKSPACE_SPOT_CORAL=/home/spot/spot_coral_ws
WORKSPACE_SPOT=/home/spot/spot_ws

function update_workspace() {
    workspace=$1
    if [ -e $workspace ]; then
        cd $workspace/src
        wstool update -t . -j . -m 60 --delete-changed-uris --continue-on-error
        catkin build -j4 --continue-on-failure
    else
        echo "Workspace $workspace not found."
    fi
}

update_workspace $WORKSPACE_SPOT_DRIVER
update_workspace $WORKSPACE_SPOT_CORAL
update_workspace $WORKSPACE_SPOT
