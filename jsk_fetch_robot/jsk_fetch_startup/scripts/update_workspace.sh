#!/usr/bin/env bash

function usage()
{
    echo "Usage: $0 [-w workspace_directory] [-h]"
}

SEND_MAIL=true
WORKSPACE=$HOME/ros/melodic/

while getopts hl:w: OPT
do
    case $OPT in
        w)
            WORKSPACE=$(cd $(dirname $OPTARG) && pwd)/$(basename $OPTARG)
            ;;
        l)
            SEND_MAIL=false
        h)
            usage
            exit 1
            ;;
    esac
done

if [ ! -e $WORKSPACE ]; then
    echo "No workspace $WORKSPACE"
    exit 1
fi

. $WORKSPACE/devel/setup.bash
# Filename should end with .txt to preview the contents in a web browser
LOGFILE=$WORKSPACE/update_workspace.txt

{
set -x
# Update workspace
wstool -t $WORKSPACE/src foreach --git 'git stash'
wstool -t $WORKSPACE/src update jsk-ros-pkg/jsk_robot
ln -sf $(rospack find jsk_fetch_startup)/../jsk_fetch.rosinstall.$ROS_DISTRO $WORKSPACE/src/.rosinstall
wstool -t $WORKSPACE/src update --delete-changed-uris
wstool -t $WORKSPACE/src foreach --git --shell 'branchname=$(git rev-parse --abbrev-ref HEAD); git reset --hard HEAD; git checkout origin/$branchname; git branch -D $branchname; git checkout $branchname' # Forcefully checkout specified branch
wstool -t $WORKSPACE/src update
WSTOOL_UPDATE_RESULT=$?
# Build workspace
cd $WORKSPACE
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
} 2>&1 | tee $LOGFILE
if [ -n "$MAIL_BODY" ] && "${SEND_MAIL}"; then
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
