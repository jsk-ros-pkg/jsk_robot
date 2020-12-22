#!/bin/bash

jsk_fetch_startup=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`"/.. > /dev/null && pwd)

IFS=':' read -r -a prefix_paths <<< "$CMAKE_PREFIX_PATH"
current_prefix_path="${prefix_paths[0]}"

#
cd $jsk_fetch_startup/udev_rules
for file in $(ls ./*.rules); do
    sudo cp $file /etc/udev/rules.d/
    sudo chown root:root /etc/udev/rules.d/$file
    sudo chmod 644 /etc/udev/rules.d/$file
    echo "copied $file to /etc/udev/rules.d/"
done
