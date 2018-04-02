#!/bin/bash

ARGS=""
for OPT in "$@"; do
  case "$OPT" in
    __*)  # remove ROS arguments
      shift
      ;;
    *)
      ARGS="$ARGS $1"
      shift
      ;;
  esac
done

roslaunch jsk_pr2_startup kinect_head_c2.launch $ARGS --screen 1>&2
