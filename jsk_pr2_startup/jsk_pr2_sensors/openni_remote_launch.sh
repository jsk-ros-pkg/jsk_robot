#!/bin/sh
set +e
MACHINE_NAME=$1
if [ "$1" = "" ]; then MACHINE_NAME="c2"; fi

ssh -t -l applications ${MACHINE_NAME} "export PS1=applications;source ~/.bashrc; roslaunch jsk_pcl_ros openni_remote.launch camera:=openni camera_remote:=camera_${MACHINE_NAME}"
