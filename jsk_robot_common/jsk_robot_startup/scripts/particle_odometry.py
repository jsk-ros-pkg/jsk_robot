#! /usr/bin/env python

import rospy
from jsk_robot_startup.ParticleOdometry import *

if __name__ == '__main__':
    try:
        node = ParticleOdometry()
        node.execute()
    except rospy.ROSInterruptException: pass
