#! /usr/bin/env python

import rospy
from jsk_robot_startup.CalculateOdomInitToBaseLinkTransform import *

if __name__ == '__main__':
    try:
        node = CalculateOdomInitToBaseLinkTransform()
        node.execute()
    except rospy.ROSInterruptException: pass
