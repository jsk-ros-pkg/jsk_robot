#!/usr/bin/env python

import sys
import rospy
import actionlib
from face_recognition.msg import FaceRecognitionAction, FaceRecognitionGoal


if __name__ == '__main__':
    rospy.init_node('face_recognition_starter')
    c = actionlib.ActionClient('face_recognition', FaceRecognitionAction)
    c.wait_for_server()
    g = FaceRecognitionGoal()
    g.order_id = 1
    g.order_argument = "none"
    c.send_goal(g)
    sys.exit(0)
