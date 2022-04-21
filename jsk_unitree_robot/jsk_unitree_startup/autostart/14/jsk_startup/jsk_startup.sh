#!/bin/sh
##############
# Author: kei
# Put BEFORE faceLightMqtt in ~/Unitree/autostart/.startlist.sh
##############
sleep 2

eval echo "[jsk_startup] starting ... " $toStartlog
./run.sh  &

