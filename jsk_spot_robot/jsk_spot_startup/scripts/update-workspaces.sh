#!/bin/bash

WORKSPACE_SPOT_DRIVER=/home/spot/spot_driver_ws
WORKSPACE_SPOT_CORAL=/home/spot/spot_coral_ws
WORKSPACE_SPOT=/home/spot/spot_ws
MAIL_BODY_FILE=/tmp/mail-body.txt

sudo apt-get update -y && sudo apt-get upgrade -y

echo "Result of workspace updating in $ROBOT_NAME\n" > $MAIL_BODY_FILE

function update_and_build () {
    local WORKSPACE=$1
    local UPDATE_LOG=$2
    local BUILD_LOG=$3
    if [ -e $WORKSPACE ]; then
        cd $WORKSPACE/src
        wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_driver.rosinstall --merge-replace --confirm-all
        wstool update -t . -j 4 -m 60 --delete-changed-uris 2>&1 | tee $UPDATE_LOG
        if [ $? -eq 0 ]; then
            echo "Updating of $WORKSPACE succeeded.\n" >> $MAIL_BODY_FILE
            echo "Updating of $WORKSPACE succeeded.\n"
        else
            echo "Updating of $WORKSPACE failed.\n" >> $MAIL_BODY_FILE
            echo "Updating of $WORKSPACE failed.\n"
        fi
        catkin build --no-status -j4 --continue-on-failure 2>&1 | tee $BUILD_LOG
        if [ $? -eq 0 ]; then
            echo "Building of $WORKSPACE succeeded.\n" >> $MAIL_BODY_FILE
            echo "Building of $WORKSPACE succeeded.\n"
        else
            echo "Building of $WORKSPACE failed.\n" >> $MAIL_BODY_FILE
            echo "Building of $WORKSPACE failed.\n"
        fi
    else
        echo "Workspace $WORKSPACE not found.\n" >> $MAIL_BODY_FILE
        echo "Workspace $WORKSPACE not found.\n"
        touch $UPDATE_LOG
        touch $BUILD_LOG
    fi
}

update_and_build $WORKSPACE_SPOT_DRIVER /tmp/update_spot_driver.log /tmp/build_spot_driver.log
update_and_build $WORKSPACE_SPOT_CORAL /tmp/update_spot_coral.log /tmp/build_spot_coral.log
update_and_build $WORKSPACE_SPOT /tmp/update_spot.log /tmp/build_spot.log

cat $MAIL_BODY_FILE | mailx \
    -A /tmp/update_spot_driver.log \
    -A /tmp/update_spot_coral.log \
    -A /tmp/update_spot.log \
    -A /tmp/build_spot_driver.log \
    -A /tmp/build_spot_coral.log \
    -A /tmp/build_spot.log \
    -s "Workspace Updating Notification from $ROBOT_NAME" -r spot-jsk@jsk.imi.i.u-tokyo.ac.jp spot@jsk.imi.i.u-tokyo.ac.jp
