#!/bin/bash
echo $0
if (echo $0|grep 'stop_sound.sh' > /dev/null); then
    echo -n "Stopping sound: "
    if type aplay &> /dev/null && [ -f "/usr/share/sounds/jsk_robot_startup/stop_sound.wav" ]; then
        aplay --device=sysdefault:CARD=AUDIO /usr/share/sounds/jsk_robot_startup/stop_sound.wav
    elif type beep &> /dev/null; then
        beep -f 783.99 -l 250 -n -f 659.26 -l 250 -n -f 587.33 -l 250 -n -f 523.25 -l 500
    fi
elif (echo $0|grep 'start_sound.sh' > /dev/null); then
    echo -n "Starting sound: "
    if type aplay &> /dev/null && [ -f "/usr/share/sounds/jsk_robot_startup/start_sound.wav" ]; then
        aplay --device=sysdefault:CARD=AUDIO /usr/share/sounds/jsk_robot_startup/start_sound.wav
    elif type beep &> /dev/null; then
        beep -f 523.25 -l 250 -n -f 587.33 -l 250 -n -f 659.26 -l 250 -n -f 783.99 -l 500
    fi
fi
