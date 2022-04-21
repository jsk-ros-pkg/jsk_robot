#!/bin/sh
##############
# Author: kei
# Put AFTER sportMode in ~/Unitree/autostart/.startlist.sh
##############
sleep 2

eval echo "[jsk_startup] starting... " $toStartlog

./soundplay_node.sh &
./rwt_app_chooser.sh &

