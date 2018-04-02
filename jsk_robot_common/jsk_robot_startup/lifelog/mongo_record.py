#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import sys
import rospy
from jsk_robot_startup.lifelog import MongoRecord


if __name__ == '__main__':
    rospy.init_node("mongo_record", anonymous=True)

    if len(sys.argv) != len(rospy.myargv()):
        # spawn by roslaunch
        rec = MongoRecord()
    else:
        # spawn by command
        rec = MongoRecord(sys.argv[1:])

    rec.run()
