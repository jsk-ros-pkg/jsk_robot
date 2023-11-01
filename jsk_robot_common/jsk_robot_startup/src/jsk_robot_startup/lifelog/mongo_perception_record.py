#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Naoto Tsukamoto <tsukamoto@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from jsk_robot_startup.lifelog import MongoRecord

from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_recognition_msgs.msg import RectArray
from opencv_apps.msg import FaceArrayStamped


class MongoPerceptionRecord(MongoRecord):
    '''
    Insert recognition topics only when they have recognition results.
    '''
    def callback(self, msg, topic):

        if isinstance(msg, ClassificationResult):
            if msg.labels:
                self.insert(msg,
                            meta={'input_topic': topic},
                            wait=self.blocking)
        elif isinstance(msg, PeoplePoseArray):
            if msg.poses:
                self.insert(msg,
                            meta={'input_topic': topic},
                            wait=self.blocking)
        elif isinstance(msg, RectArray):
            if msg.rects:
                self.insert(msg,
                            meta={'input_topic': topic},
                            wait=self.blocking)
        elif isinstance(msg, FaceArrayStamped):
            if msg.faces:
                self.insert(msg,
                            meta={'input_topic': topic},
                            wait=self.blocking)


if __name__ == '__main__':
    import sys
    rospy.init_node("perception_record", anonymous=True)

    if len(sys.argv) != len(rospy.myargv()):
        # spawn by roslaunch
        rec = MongoPerceptionRecord()
    else:
        # spawn by command
        rec = MongoPerceptionRecord(sys.argv[1:])

    rec.run()
