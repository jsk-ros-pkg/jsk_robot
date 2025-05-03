#!/bin/bash

source $(rospack find jsk_panda_startup)/scripts/dot.bash_byobu_function

byobu-kill-window franka roscore

byobu-kill-window franka main

byobu-kill-window franka teleop

byobu-kill-window franka rviz

byobu-kill-window franka coral

byobu-kill-session franka
