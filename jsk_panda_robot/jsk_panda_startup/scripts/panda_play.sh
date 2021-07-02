#!/usr/bin/env bash

OPTIONS=""
FILENAMES=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -*)
      OPTIONS="$OPTIONS $1"
      shift
      ;;
    *)
      FILENAMES="$FILENAMES $(readlink -f $1)"
      shift
      ;;
  esac
done

roslaunch jsk_panda_startup panda_play.launch bagfile_name:="$FILENAMES" rosbag_option:="$OPTIONS"
