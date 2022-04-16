#!/usr/bin/env bash

. $HOME/ros/melodic/devel/setup.bash
# Filename should end with .txt to preview the contents in a web browser
LOGFILE=$HOME/ros/melodic/update_workspace.txt

{
set -x
# Update workspace
cd $HOME/ros/melodic/src
ln -sf $(rospack find jsk_fetch_startup)/../jsk_fetch.rosinstall.$ROS_DISTRO $HOME/ros/melodic/src/.rosinstall
wstool foreach --git 'git stash'
wstool update --delete-changed-uris
wstool foreach --git 'branch-name=$(git rev-parse --abbrev-ref HEAD) && git reset --hard HEAD && git checkout origin/$branch-name && git branch -D $branch-name && git checkout $branch-name' # Forcefully checkout specified branch
wstool update
WSTOOL_UPDATE_RESULT=$?
# Build workspace
cd $HOME/ros/melodic
catkin clean aques_talk collada_urdf_jsk_patch libcmt -y
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
CATKIN_BUILD_RESULT=$?
# Send mail
MAIL_BODY=""
if [ $WSTOOL_UPDATE_RESULT -ne 0 ]; then
    MAIL_BODY=$MAIL_BODY"Please wstool update workspace manually. "
fi
if [ $CATKIN_BUILD_RESULT -ne 0 ]; then
    MAIL_BODY=$MAIL_BODY"Please catkin build workspace manually."
fi
set +x
} > $LOGFILE 2>&1
if [ -n "$MAIL_BODY" ]; then
   rostopic pub -1 /email jsk_robot_startup/Email "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
subject: 'Daily workspace update fails'
body: '$MAIL_BODY'
sender_address: '$(hostname)@jsk.imi.i.u-tokyo.ac.jp'
receiver_address: 'fetch@jsk.imi.i.u-tokyo.ac.jp'
smtp_server: ''
smtp_port: ''
attached_files: ['$LOGFILE']"
fi
