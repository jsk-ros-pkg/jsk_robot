#!/bin/bash

WORKSPACE_SPOT_DRIVER=/home/spot/spot_driver_ws
WORKSPACE_SPOT_CORAL=/home/spot/spot_coral_ws
WORKSPACE_SPOT=/home/spot/spot_ws
MAIL_BODY_FILE=/tmp/mail-body.txt

sudo apt-get update -y && sudo apt-get upgrade -y

echo "Result of workspace updating\n" > $MAIL_BODY_FILE

if [ -e $WORKSPACE_SPOT_DRIVER ]; then
    cd $WORKSPACE_SPOT_DRIVER/src
    wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_driver.rosinstall --merge-replace --confirm-all
    wstool update -t . -j 4 -m 60 --delete-changed-uris --continue-on-error
    catkin build -v --pre-clean --no-status -j4 --continue-on-failure
    if [ $? -eq 0 ]; then
        echo "Updating and building of $WORKSPACE_SPOT_DRIVER succeeded.\n" >> $MAIL_BODY_FILE
    else
        echo "Updating and building of $WORKSPACE_SPOT_DRIVER failed.\n" >> $MAIL_BODY_FILE
    fi
else
    echo "Workspace $WORKSPACE_SPOT_DRIVER not found.\n" >> $MAIL_BODY_FILE
fi

if [ -e $WORKSPACE_SPOT_CORAL ]; then
    cd $WORKSPACE_SPOT_CORAL/src
    wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_coral.rosinstall --merge-replace --confirm-all
    wstool update -t . -j 4 -m 60 --delete-changed-uris --continue-on-error
    catkin build -v --pre-clean --no-status -j4 --continue-on-failure
    if [ $? -eq 0 ]; then
        echo "Updating and building of $WORKSPACE_SPOT_CORAL succeeded.\n" >> $MAIL_BODY_FILE
    else
        echo "Updating and building of $WORKSPACE_SPOT_CORAL failed.\n" >> $MAIL_BODY_FILE
    fi
else
    echo "Workspace $WORKSPACE_SPOT_CORAL not found.\n" >> $MAIL_BODY_FILE
fi

if [ -e $WORKSPACE_SPOT ]; then
    cd $WORKSPACE_SPOT/src
    wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_dev.rosinstall --merge-replace --confirm-all
    wstool update -t . -j 4 -m 60 --delete-changed-uris --continue-on-error
    catkin build -v --pre-clean --no-status -j4 --continue-on-failure
    if [ $? -eq 0 ]; then
        echo "Updating and building of $WORKSPACE_SPOT succeeded.\n" >> $MAIL_BODY_FILE
    else
        echo "Updating and building of $WORKSPACE_SPOT failed.\n" >> $MAIL_BODY_FILE
    fi
else
    echo "Workspace $WORKSPACE_SPOT not found.\n" >> $MAIL_BODY_FILE
fi

cat $MAIL_BODY_FILE

cat $MAIL_BODY_FILE | mail -s "Workspace Updating Notification" -r spot-jsk@jsk.imi.i.u-tokyo.ac.jp spot@jsk.imi.i.u-tokyo.ac.jp
