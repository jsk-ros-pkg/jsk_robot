#! /usr/bin/env python

import rospy
from jsk_robot_startup.EKFGPFOdometry import *

if __name__ == '__main__':
    try:
        node = EKFGPFOdometry()
        node.execute()
    except rospy.ROSInterruptException: pass
