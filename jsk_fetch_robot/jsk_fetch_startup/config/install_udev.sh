#!/bin/bash

# 99-edgetpu-accelerator.rules and 99-realsense-libusb.rules are not tracked in jsk_fetch_startup
# because they are installed by apt

jsk_fetch_startup=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`"/.. > /dev/null && pwd)

cd $jsk_fetch_startup/udev_rules
for file in $(ls *.rules); do
    sudo ln -sf $jsk_fetch_startup/udev_rules/$file /etc/udev/rules.d/$file
    echo -e "Create symbolic link from jsk_fetch_startup/udev_rules/$file to /etc/udev/rules.d/$file\n"
done
