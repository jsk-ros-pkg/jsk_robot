#!/usr/bin/env python

# some joints must be taken w/ care! these joints are initiated once the ROS controller is closed

import subprocess
from seed_command import SeedCommand

import sys
import os
import time

R_HAND_Y = 13
L_HAND_Y = 28

time.sleep(30.0) # wait controller launch

pid = os.getpid()
launch_killed_before_check = True # a quick kill may indicate something unusual, don't send commands

print '---ready for safe exit'

while True:
    status = int(subprocess.check_output("ps aux | grep aero_ros_controller | wc -l", shell=True)[:-1])
    print '---check safe once %d' % status
    if status < 3:
        print '---detected controller kill!'
        if not launch_killed_before_check: # init pose at end if controller was correctly launched
            seed = SeedCommand('aero_upper', 1000000)
            seed.AERO_Snd_Position(R_HAND_Y, 100, 0) #ID, time[*10msec], stroke[*0.01mm]
            seed.AERO_Snd_Position(L_HAND_Y, 100, 0) #ID, time[*10msec], stroke[*0.01mm]
        break
    else:
        launch_killed_before_check = False
        time.sleep(2.0)
subprocess.check_output("kill " + str(pid), shell=True)
sys.exit()
