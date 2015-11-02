#! /usr/bin/env python

import rospy
from jsk_robot_startup.OdometryOffset import *

if __name__ == '__main__':
    try:
        node = OdometryOffset()
        node.execute()
    except rospy.ROSInterruptException: pass
