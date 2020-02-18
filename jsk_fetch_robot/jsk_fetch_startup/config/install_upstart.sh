#!/bin/bash

jsk_fetch_startup=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`"/.. > /dev/null && pwd)

IFS=':' read -r -a prefix_paths <<< "$CMAKE_PREFIX_PATH"
current_prefix_path="${prefix_paths[0]}"

# jsk-fetch-startup
sudo bash -c "cat <<EOF > /etc/init/jsk-fetch-startup.conf
description \"spawn jsk bringup nodes for fetch\"
# use default log dir: yamaguchi & s-kitagawa (2019/04/18)
# env ROS_LOG_DIR=/var/log/ros

start on roscore_is_up
stop on roscore_is_down

respawn

# add ROSCONSOLE_FORMAT: s-kitagawa (2019/10/03)
env AUDIO_DEVICE=alsa_output.usb-1130_USB_AUDIO-00-AUDIO.analog-stereo
env ROSCONSOLE_FORMAT='[${severity}] [${time}] [${node}:${logger}]: ${message}'

# enable usb speaker if available
pre-start script
    # launch by fetch user: yamaguchi & s-kitagawa (2019/04/18)
    # exec su ros -c 'pactl set-default-sink $AUDIO_DEVICE || true'
    exec su fetch -c 'pactl set-default-sink $AUDIO_DEVICE || true'
end script

script
    # launch by fetch user: yamaguchi & s-kitagawa (2019/04/18)
    # exec su ros -c \". ${current_prefix_path}/setup.bash && roslaunch ${jsk_fetch_startup}/launch/fetch_bringup.launch boot_sound:=true\"
    exec su fetch -c \". ${current_prefix_path}/setup.bash && roslaunch ${jsk_fetch_startup}/launch/fetch_bringup.launch boot_sound:=true\"
end script

EOF"
echo "== cat /etc/init/jsk-fetch-startup.conf =="
cat /etc/init/jsk-fetch-startup.conf
echo "== usage: sudo service jsk-fetch-startup restart =="

# vertical-touchscreen
sudo bash -c "cat <<EOF > /etc/init/vertical-touchscreen.conf
# See https://askubuntu.com/questions/507496/how-to-start-gui-application-with-upstart
description \"Use vertical touchscreen\"

start on runlevel [2345]
stop on runlevel [016]

script
    sleep 30 # Wait for X programs to start
    export DISPLAY=:0
    exec su fetch -c \"${jsk_fetch_startup}/scripts/vertical-touchscreen.sh\"
end script

EOF"
echo
echo "== cat /etc/init/vertical-touchscreen.conf =="
cat /etc/init/vertical-touchscreen.conf
echo "== usage: sudo service vertical-touchscreen restart =="
