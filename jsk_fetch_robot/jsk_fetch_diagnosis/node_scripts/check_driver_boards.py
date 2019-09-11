#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import subprocess
import sys

import rospy

from jsk_fetch_diagnosis.msg import BoardInfo

list_board_id = [ 0,   # mainboard
                  17,  # l_wheel
                  18,  # r_wheel
                  19,  # torso_lift
                  20,  # head_pan
                  21,  # heal_tilt
                  #35,  # ?? cart_dock_mcb
                  38,  # shoulder_pan
                  39,  # shoulder_lift
                  40,  # upperarm_roll
                  41,  # elbow_flex
                  42,  # forearm_roll
                  43,  # wrist_flex
                  44,  # wrist_roll
                  #81,  # ?? mcb Base motor test stand
                  #82,  # ?? mcb Base motor test stand
                  63,  # charger 
                  128] # gripper

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument( '--topicname',
                         help='topicname of result of read_board commands',
                         default='/check_board_info' )
    args = parser.parse_args()

    topicname = args.topicname

    binary_str = '/opt/ros/' + os.environment['ROS_DISTRO'] + '/lib/fetch_drivers/read_board'

    rospy.init_node( 'check_board_info' )
    publisher = rospy.Publisher( topicname, BoardInfo, queue_size=10 )
    rate = rospy.Rate( 1 )

    while not rospy.is_shutdown():

        rospy.loginfo( 'read data from motor driver boards and publish them to ' + topicname + ' topic.' )

        for board_id in list_board_id:
            msg = BoardInfo()
            msg.header.stamp = rospy.Time.now()
            msg.board_id = board_id
            msg.board_info = subprocess.check_output( [ binary_str, str( board_id ) ] )
            publisher.publish( msg )

        rate.sleep()
