#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import argparse
from baxter_core_msgs.msg import AssemblyState
import baxter_interface
from baxter_interface import CHECK_VERSION
from multiprocessing import Pool
from rosgraph_msgs.msg import Clock
import rospy
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--moveit', '-m', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('initialize_baxter')

    rospy.loginfo('waiting for /clock & /robot/state')
    rospy.wait_for_message('/clock', Clock)
    rospy.wait_for_message('/robot/state', AssemblyState)
    rospy.loginfo('found /clock & /robot/state')

    # enable robot
    baxter_interface.RobotEnable(CHECK_VERSION).enable()

    # joint action server
    pool = Pool(processes=3)
    commands = [
        ('rosrun', 'baxter_interface', 'joint_trajectory_action_server.py'),
        ('rosrun', 'baxter_interface', 'head_action_server.py')]
    if args.moveit:
        commands.append((
            'roslaunch', 'baxter_moveit_config', 'move_group.launch'))
    pool.map(subprocess.call, commands)


if __name__ == '__main__':
    main()
