#!/usr/bin/python
#
# Store the ObjectDetection message
#

import rospy
from jsk_robot_startup.lifelog import ObjectDetectionLogger


if __name__ == "__main__":
    rospy.init_node('object_detecton_logger')
    ObjectDetectionLogger().run()
