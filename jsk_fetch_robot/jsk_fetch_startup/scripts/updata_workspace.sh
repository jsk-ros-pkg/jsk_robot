#!/usr/bin/env bash

. $HOME/ros/melodic/devel/setup.bash

set -x

cd $HOME/ros/melodic/src
ln -sf $(rospack find jsk_fetch_startup)/../jsk_fetch.rosinstall.$ROS_DISTRO $HOME/ros/melodic/src/.rosinstall
wstool update --delete-changed-uris --continue-on-error
catkin build
set +x
