#!/usr/bin/env bash

. $HOME/ros/melodic/devel/setup.bash

set -x

cd $HOME/ros/melodic/src
ln -sf $(rospack find jsk_fetch_startup)/../jsk_fetch.rosinstall.$ROS_DISTRO $HOME/ros/melodic/src/.rosinstall
wstool foreach --git 'git stash'
wstool update --delete-changed-uris
if [ $? -ne 0 ]; then
    LC_CTYPE=en_US.UTF-8 /bin/echo -e "Please update workspace manually" | /usr/bin/mail -s "Daily wstool update fails" -r $(hostname)@jsk.imi.i.u-tokyo.ac.jp fetch@jsk.imi.i.u-tokyo.ac.jp
fi
cd $HOME/ros/melodic
catkin clean aques_talk collada_urdf_jsk_patch -y
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
set +x
