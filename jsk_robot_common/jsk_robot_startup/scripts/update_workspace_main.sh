#!/usr/bin/env bash

function usage()
{
    echo "Usage: $0 [-w workspace_directory] [-r rosinstall_path] [-t robot_type] [-s skip keys] [-h] [-u] [-l] [-n]

optional arguments:
    -h                  show this help
    -w WORKSPACE_PATH   specify target workspace
    -r ROSINTALL_PATH   rosinstall path
    -t ROBOT            robot type
    -s SKIP_KEYS        rosdep install skip keys
    -u                  do not run apt-get upgrade and rosdep install
    -l                  do not send a mail
    -n                  do not update workspace, only show wstool status
"
}

function get_full_path()
{
    echo "$(cd $(dirname $1) && pwd)/$(basename $1)"
}

ROSDEP_INSTALL=true
SEND_MAIL=true
UPDATE_WORKSPACE=true
WORKSPACE=$(get_full_path $HOME/ros/$ROS_DISTRO)

while getopts w:r:t:s:ulnh OPT
do
    case $OPT in
        w)
            WORKSPACE=$(get_full_path $OPTARG)
            ;;
        r)
            ROSINSTALL=$(get_full_path $OPTARG)
            ;;
        t)
            ROBOT_TYPE=$OPTARG
            ;;
        s)
            SKIP_KEYS=$OPTARG
            ;;
        u)
            ROSDEP_INSTALL=false
            ;;
        l)
            SEND_MAIL=false
            ;;
        n)
            UPDATE_WORKSPACE=false
            ;;
        h)
            usage
            exit 1
            ;;
        *)
            usage
            exit 1
            ;;
    esac
done

if [ "$WORKSPACE" = "" ]; then
    echo "Please set valid workspace -w $WORKSPACE<br>"
    exit 1
fi
if [ "$ROSINSTALL" = "" ]; then
    echo "Please set valid rosinstall -r $ROSINSTALL<br>"
    exit 1
fi
if [ "$ROBOT_TYPE" = "" ]; then
    echo "Please set valid robot type -t $ROBOT_TYPE<br>"
    exit 1
fi


. $WORKSPACE/devel/setup.bash
# Filename should end with .txt to preview the contents in a web browser
LOGFILE=$WORKSPACE/update_workspace.txt

TMP_MAIL_BODY_FILE=/tmp/update_workspace_mailbody.txt

{
set -x
# Update workspace
echo "" > $TMP_MAIL_BODY_FILE
wstool foreach -t $WORKSPACE/src --git 'git fetch origin --prune'
WSTOOL_STATUS=$(wstool status -t $WORKSPACE/src)
if [ -n "$WSTOOL_STATUS" ]; then
    echo -e "<b>Please commit robot internal change and send pull request.</b><br><br>" >> $TMP_MAIL_BODY_FILE
    echo -e $WSTOOL_STATUS >> $TMP_MAIL_BODY_FILE
    # escape " ' , -- and add change line code to end of line
    wstool diff -t $WORKSPACE/src | sed -e "s/'/ /g" -e "s/^--/ /g" -e 's/"/ /g' -e "s/<br>/\\\n/" -e 's/$/<br>/g' -e "s/,/ /g" | tee -a $TMP_MAIL_BODY_FILE
fi
if [ "${UPDATE_WORKSPACE}" == "true" ]; then
    wstool foreach -t $WORKSPACE/src --git 'git stash -u'
    wstool foreach -t $WORKSPACE/src --git 'git fetch origin --prune'
    wstool update -t $WORKSPACE/src $(rospack find jsk_robot_startup)/../.. --delete-changed-uris
    ln -sf $ROSINSTALL $WORKSPACE/src/.rosinstall
    wstool update -t $WORKSPACE/src --delete-changed-uris
    # When the repository's Spec-Version branch has commits which are not pushed to remote, they are evacuated to another branch
    wstool foreach -t $WORKSPACE/src --git --shell 'expected_version=$(wstool info . | grep Spec-Version: | awk '\''{print $2}'\''); current_version=$(git rev-parse --abbrev-ref HEAD); remote_diff_head_origin=$(git rev-list HEAD..origin); remote_diff_origin_head=$(git rev-list origin..HEAD); remote_diff="${remote_diff_head_origin}${remote_diff_origin_head}"; if [ "$expected_version" = "$current_version" ] && [ -n "$remote_diff" ]; then git checkout -b ${expected_version}-patch-$(date +%Y%m%d%H%M%S); git checkout origin/$expected_version; git branch -D $expected_version; git checkout -b $expected_version --track origin/$expected_version; fi'
    wstool update -t $WORKSPACE/src
    WSTOOL_UPDATE_RESULT=$?
else
    WSTOOL_UPDATE_RESULT=0
fi
# Rosdep Install
if [ "${ROSDEP_INSTALL}" == "true" ]; then
  sudo apt-get update -y;
  rosdep update;
  rosdep install --from-paths $WORKSPACE/src --ignore-src -y -r --skip-keys "$SKIP_KEYS";
  ROSDEP_INSTALL_RESULT=$?;
else
  ROSDEP_INSTALL_RESULT=0;
fi
# Build workspace
cd $WORKSPACE
catkin clean aques_talk collada_urdf_jsk_patch libcmt -y
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build --continue-on-failure
CATKIN_BUILD_RESULT=$?
# Send mail
if [ $WSTOOL_UPDATE_RESULT -ne 0 ]; then
    echo "<b>Please wstool update workspace manually.</b><br>" >> $TMP_MAIL_BODY_FILE
fi
if [ $ROSDEP_INSTALL_RESULT -ne 0 ]; then
    echo "<b>Please install dependencies manually.</b><br>" >> $TMP_MAIL_BODY_FILE
fi
if [ $CATKIN_BUILD_RESULT -ne 0 ]; then
    echo "<b>Please catkin build workspace manually.</b><br>" >> $TMP_MAIL_BODY_FILE
fi
set +x
} 2>&1 | tee $LOGFILE

# MAIL_BODY variable cannot be set directly in a subshell. So it is set from temporary mail body text file.
# The mail body text is put as $TMP_MAIL_BODY_FILE.
# See https://github.com/jsk-ros-pkg/jsk_robot/issues/1569
MAIL_BODY=$(cat $TMP_MAIL_BODY_FILE)
echo "MAIL_BODY: $MAIL_BODY"

if [ -n "$MAIL_BODY" ] && [ "${SEND_MAIL}" == "true" ]; then
    echo "Sent a mail"
    rostopic pub -1 /email jsk_robot_startup/Email "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
subject: 'Daily workspace update fails'
body:
- {type: 'html', message: '${MAIL_BODY}', file_path: '', img_data: '', img_size: 0}
sender_address: '$(hostname)@jsk.imi.i.u-tokyo.ac.jp'
receiver_address: '$ROBOT_TYPE@jsk.imi.i.u-tokyo.ac.jp'
smtp_server: ''
smtp_port: ''
attached_files: ['$LOGFILE']"
fi
