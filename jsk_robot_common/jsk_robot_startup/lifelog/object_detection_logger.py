#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
#
# Store the ObjectDetection message
#

import rospy
from jsk_robot_startup.lifelog import ObjectDetectionLogger


if __name__ == "__main__":
    rospy.init_node('object_detecton_logger')
    ObjectDetectionLogger().run()
