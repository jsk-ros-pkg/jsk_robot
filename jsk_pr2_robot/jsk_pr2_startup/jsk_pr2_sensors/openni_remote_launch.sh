#!/bin/sh
# set +e
MACHINE_NAME=$1
if [ "$1" = "" ]; then MACHINE_NAME="c2"; fi

source ~/.bashrc
roslaunch jsk_pr2_startup openni_remote.launch MACHINE_NAME:=${MACHINE_NAME}

