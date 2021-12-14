#!/usr/bin/env bash

. $HOME/ros/melodic/devel/setup.bash

set -x
# Update workspace
cd $HOME/ros/melodic/src
ln -sf $(rospack find jsk_fetch_startup)/../jsk_fetch.rosinstall.$ROS_DISTRO $HOME/ros/melodic/src/.rosinstall
wstool foreach --git 'git stash'
wstool update --delete-changed-uris
WSTOOL_UPDATE_RESULT=$?
cd $HOME/ros/melodic
catkin clean aques_talk collada_urdf_jsk_patch -y
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
CATKIN_BUILD_RESULT=$?
# Send mail
MAIL_BODY=""
if [ $WSTOOL_UPDATE_RESULT -ne 0 ]; then
    MAIL_BODY=$MAIL_BODY"Please wstool update workspace manually\n"
fi
if [ $CATKIN_BUILD_RESULT -ne 0 ]; then
    MAIL_BODY=$MAIL_BODY"Please catkin build workspace manually\n"
fi
LC_CTYPE=en_US.UTF-8 /bin/echo -e $MAIL_BODY | /usr/bin/mail -s "Daily workspace update fails" -r $(hostname)@jsk.imi.i.u-tokyo.ac.jp fetch@jsk.imi.i.u-tokyo.ac.jp

set +x
