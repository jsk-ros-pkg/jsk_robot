#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import subprocess
import sys

import rospy

from jsk_fetch_diagnosis.msg import BoardInfo

binary_str = '/opt/ros/' + os.environ['ROS_DISTRO'] + '/lib/fetch_drivers/read_board'
list_board_id = {0:   'mainboard',
                 17:  'l_wheel',
                 18:  'r_wheel',
                 19:  'torso_lift',
                 20:  'head_pan',
                 21:  'heal_tilt',
                 # 35:  '?? cart_dock_mcb',
                 38:  'shoulder_pan',
                 39:  'shoulder_lift',
                 40:  'upperarm_roll',
                 41:  'elbow_flex',
                 42:  'forearm_roll',
                 43:  'wrist_flex',
                 44:  'wrist_roll',
                 # 81:  '?? mcb Base motor test stand',
                 # 82:  '?? mcb Base motor test stand',
                 63:  'charger',
                 128: 'gripper',
                 }


class CheckDriverBoardsNode:

    def __init__(self):

        self.publisher = rospy.Publisher('~output', BoardInfo, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.callback)

    def callback(self, event):

        rospy.logdebug('read data from motor driver boards and publish')

        for board_id in list_board_id:
            msg = BoardInfo()
            msg.header.stamp = rospy.Time.now()
            msg.board_id = board_id
            try:
                msg.board_info = subprocess.check_output([binary_str, str(board_id)])
                rospy.logdebug('read_board command with id {} succeeded'.format(board_id))
            except subprocess.CalledProcessError:
                msg.board_info = 'ERROR: command returned non-zero exit status'
                rospy.logerr(
                    'command returned non-zero exit status while read board with id '\
                    '{:#04x}: {}'.format(board_id, list_board_id[board_id]))

            self.publisher.publish(msg)


if __name__ == '__main__':

    rospy.init_node('check_board_info')
    cdbn = CheckDriverBoardsNode()
    rospy.spin()
