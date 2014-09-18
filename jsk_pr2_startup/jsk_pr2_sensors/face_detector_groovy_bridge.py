#!/usr/bin/env python

# simple script to convert from people_msgs/PositionMeasurementArray to people_msgs/PositionMeasurement

import os
import rospy

try:
    from people_msgs.msg import *
except:
    import roslib; roslib.load_manifest("people_msgs")
    from people_msgs.msg import *

def arrayCallback(msg):
    global repub
    if len(msg.people) > 0:
        repub.publish(msg.people[0])


def main():
    global repub
    repub = rospy.Publisher("/face_detector/people_tracker_measurements",
                            PositionMeasurement)
    s = rospy.Subscriber("/face_detector/people_tracker_measurements_array",
                         PositionMeasurementArray,
                         arrayCallback)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("face_detector_groovy_bridge")
    if os.environ["ROS_DISTRO"] == "groovy":
        rospy.logfatal("face_detector_groovy_bridge cannot run on groovy")
        exit(1)
    else:
        main()


