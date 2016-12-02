#!/usr/bin/env python

# automatically reset slam as robot put on the ground

import rospy
from std_msgs.msg import Empty
import subprocess

def callback(msg):
    rospy.loginfo("Killing /gmapping_node")
    subprocess.check_call(["rosnode", "kill", "/gmapping_node"])

if __name__ == "__main__":
    rospy.init_node("auto_reset_slam")
    sub = rospy.Subscriber("/odom_init_trigger", Empty, callback)
    rospy.spin()
