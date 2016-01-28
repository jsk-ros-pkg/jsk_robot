#!/bin/bash

jsk_fetch_startup=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`"/.. > /dev/null && pwd)

IFS=':' read -r -a prefix_paths <<< "$CMAKE_PREFIX_PATH"
current_prefix_path="${prefix_paths[0]}"

sudo bash -c "cat <<EOF > /etc/init/jsk-fetch-startup.conf

env ROS_LOG_DIR=/var/log/ros

start on roscore_is_up
stop on roscore_is_down

respawn

script
    exec su ros -c \". ${current_prefix_path}/setup.bash && roslaunch ${jsk_fetch_startup}/launch/fetch_bringup.launch boot_sound:=true\"
end script

EOF"
echo "== cat /etc/init/jsk-fetch-startup.conf =="
cat /etc/init/jsk-fetch-startup.conf
echo "== usage: sudo service jsk-fetch-startup restart =="
