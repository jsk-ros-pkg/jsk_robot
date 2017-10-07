#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
#
# This script stores actionlib goals, results and feedbacks by MongoDB
# And store the current robot joints when recieve result.
#
# table configuration is bottom of this script
#

# parameters:
#    joint_tolerance: store the past joint_states (sec)

import rospy
from jsk_robot_startup.lifelog import ActionLogger


if __name__ == "__main__":
    rospy.init_node('action_logger')
    ActionLogger().run()
