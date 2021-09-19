#!/usr/bin/env bash

# link to latest calibrated urdf: s-kitagawa (2020/02/21)
# link to latest calibrated rgb and depth yaml: s-kitagawa (2020/09/18)
if [ -e $FETCH_CALIBRATED_URDF ]; then
  ln -sf $FETCH_CALIBRATED_URDF /etc/ros/$ROS_DISTRO/calibrated_latest.urdf
  echo "linked to calibrated URDF: $FETCH_CALIBRATED_URDF -> /etc/ros/$ROS_DISTRO/calibrated_latest.urdf"
fi
if [ -e $FETCH_CALIBRATED_DEPTH ]; then
  ln -sf $FETCH_CALIBRATED_DEPTH /etc/ros/$ROS_DISTRO/depth_latest.yaml
  echo "linked to calibrated depth yaml: $FETCH_CALIBRATED_DEPTH -> /etc/ros/$ROS_DISTRO/depth_latest.yaml"
fi
if [ -e $FETCH_CALIBRATED_RGB ]; then
  ln -sf $FETCH_CALIBRATED_RGB /etc/ros/$ROS_DISTRO/rgb_latest.yaml
  echo "linked to calibrated rgb yaml: $FETCH_CALIBRATED_RGB -> /etc/ros/$ROS_DISTRO/rgb_latest.yaml"
fi

