#!/usr/bin/env bash

cp $(rospack find jsk_robot_startup)/scripts/update_workspace_main.sh /tmp/update_workspace.sh
/tmp/update_workspace.sh -w $HOME/ros/indigo -r $(rospack find jsk_pr2_startup)/jsk_pr2.rosinstall -t pr2 -u
rm /tmp/update_workspace.sh
