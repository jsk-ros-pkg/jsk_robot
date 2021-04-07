#!/usr/bin/env bash

. $HOME/ros/melodic/devel/setup.bash

set -x

cd $HOME/ros/melodic/src
ln -sf $(rospack find jsk_fetch_startup)/../jsk_fetch.rosinstall.$ROS_DISTRO $HOME/ros/melodic/src/.rosinstall
wstool foreach --git 'git stash'
wstool update --delete-changed-uris
cd $HOME/ros/melodic
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build -DCMAKE_BUILD_TYPE=Release
set +x
