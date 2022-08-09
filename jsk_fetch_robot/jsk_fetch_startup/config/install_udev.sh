#!/bin/bash

jsk_fetch_startup=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`"/.. > /dev/null && pwd)

cd $jsk_fetch_startup/udev_rules
for file in $(ls *.rules); do
    if [ -e /etc/udev/rules.d/$file ]; then
        file_bk=$file.$(date "+%Y%m%d_%H%M%S")
        sudo cp /etc/udev/rules.d/$file /etc/udev/rules.d/$file_bk
        echo "backup /etc/udev/rules.d/$file to /etc/udev/rules.d/$file_bk"
    fi

    sudo cp $file /etc/udev/rules.d/
    sudo chown root:root /etc/udev/rules.d/$file
    sudo chmod 644 /etc/udev/rules.d/$file
    echo -e "copied jsk_fetch_startup/udev_rules/$file to /etc/udev/rules.d/$file\n"
done
