#! /usr/bin/env python

import rospy
from jsk_robot_startup.OdometryFeedbackWrapper import *

if __name__ == '__main__':
    try:
        node = OdometryFeedbackWrapper()
        node.execute()
    except rospy.ROSInterruptException: pass
