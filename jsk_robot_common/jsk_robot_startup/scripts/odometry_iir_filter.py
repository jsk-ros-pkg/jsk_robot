#! /usr/bin/env python

import rospy
from jsk_robot_startup.OdometryIIRFilter import *

if __name__ == '__main__':
    try:
        node = OdometryIIRFilter()
        node.execute()
    except rospy.ROSInterruptException: pass
