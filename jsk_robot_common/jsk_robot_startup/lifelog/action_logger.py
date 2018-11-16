#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
#
# This script stores actionlib goals, results and feedbacks by MongoDB

import rospy
from jsk_robot_startup.lifelog import ActionLogger


if __name__ == "__main__":
    rospy.init_node('action_logger')
    ActionLogger().run()
