#!/usr/bin/env python

# simple script to convert from people_msgs/PositionMeasurement to people_msgs/PositionMeasurementArray

import os
import rospy

try:
    from people_msgs.msg import *
except:
    import roslib; roslib.load_manifest("people_msgs")
    from people_msgs.msg import *

def measurementCallback(msg):
    global repub
    array = PositionMeasurementArray()
    array.header = msg.header
    array.people = [msg]
    repub.publish(array)


def main():
    global repub
    repub = rospy.Publisher("/face_detector/people_tracker_measurements_array",
                            PositionMeasurementArray)
    s = rospy.Subscriber("/face_detector/people_tracker_measurements",
                         PositionMeasurement,
                         measurementCallback)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("face_detector_hydro_bridge")
    if os.environ["ROS_DISTRO"] == "groovy":
        rospy.logfatal("face_detector_groovy_bridge cannot run on groovy")
        exit(1)
    else:
        main()


