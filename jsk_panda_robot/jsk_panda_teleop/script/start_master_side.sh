#!/bin/bash

function init_device() {
    echo "initializing device..."
    echo "giving access to /dev/tty0 and /dev/tty1"
    # TODO: add udev rule to remove sudo's
    sudo chmod 777 /dev/ttyACM0 && sudo chmod 777 /dev/ttyACM1
}

function start_master_launch () {
    roslaunch jsk_panda_teleop start_panda_teleop_master_side.launch
}

init_device
start_master_launch
