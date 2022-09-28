#!/bin/bash

jsk_robot_startup=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`"/.. > /dev/null && pwd)

# Copy scripts
cd $jsk_robot_startup/systemd
if [ ! -d /usr/local/bin/jsk_robot_startup ]; then
  sudo mkdir -p /usr/local/bin/jsk_robot_startup
fi
for file in $(ls *.sh); do
    sudo cp $file /usr/local/bin/jsk_robot_startup
    sudo chown root:root /usr/local/bin/jsk_robot_startup/$file
    sudo chmod 755 /usr/local/bin/jsk_robot_startup/$file
    echo "copied jsk_robot_startup/systemd/$file to /usr/local/bin/jsk_robot_startup/$file"
done
sudo ln -sf /usr/local/bin/jsk_robot_startup/playsound.sh /usr/local/bin/jsk_robot_startup/start_sound.sh
sudo ln -sf /usr/local/bin/jsk_robot_startup/playsound.sh /usr/local/bin/jsk_robot_startup/stop_sound.sh

# Enable systemd service
for file in $(ls *.service); do
    sudo cp $file /etc/systemd/system
    sudo chown root:root /etc/systemd/system/$file
    sudo chmod 644 /etc/systemd/system/$file
    echo "copied jsk_robot_startup/systemd/$file to /etc/systemd/system/$file"
    sudo systemctl enable $file
done

# Default sound
# Created by VOICEVOX:四国めたん
cd $jsk_robot_startup/data
if [ ! -d /usr/share/sounds/jsk_robot_startup ]; then
  sudo mkdir -p /usr/share/sounds/jsk_robot_startup
fi
sudo cp start_sound.wav /usr/share/sounds/jsk_robot_startup/
sudo cp stop_sound.wav /usr/share/sounds/jsk_robot_startup/
