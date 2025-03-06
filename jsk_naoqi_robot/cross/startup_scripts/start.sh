#!/bin/bash

set -xef -o pipefail

source User/user_setup.bash
screen -c User/screenrc -dmS session bash
screen -c User/screenrc -S session -p 0 -X stuff "roslaunch jsk_pepper_startup jsk_pepper_startup.launch launch_dashboard:=false network_interface:=wlan0 launch_joy:=false^M"
sleep 1 # wait for screen to startup
screen -c User/screenrc -r
