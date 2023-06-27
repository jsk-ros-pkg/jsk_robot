#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Naoto Tsukamoto <tsukamoto@jsk.imi.i.u-tokyo.ac.jp>

import sys
import rospy
from jsk_robot_startup.lifelog import MongoPerceptionRecord

if __name__ == '__main__':
    rospy.init_node("perception_record", anonymous=True)

    if len(sys.argv) != len(rospy.myargv()):
        # spawn by roslaunch
        rec = MongoPerceptionRecord()
    else:
        # spawn by command
        rec = MongoPerceptionRecord(sys.argv[1:])

    rec.run()
