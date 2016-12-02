#!/usr/bin/env python

# automatically reset heightmap integration as robot put on the ground

import rospy
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
import subprocess

def callback(msg):
    rospy.loginfo("Clearing /accumulated_heightmap/reset")
    srv = rospy.ServiceProxy("/accumulated_heightmap/reset", EmptySrv)
    try:
        srv()
    except:
        rospy.logerr("Failed to reset /accumulated_heightmap")
    srv2 = rospy.ServiceProxy("/octomap_server/reset", EmptySrv)
    try:
        srv2()
    except:
        rospy.logerr("Failed to reset /octomap_server")

if __name__ == "__main__":
    rospy.init_node("auto_reset_heightmap")
    sub = rospy.Subscriber("/odom_init_trigger", EmptyMsg, callback)
    rospy.spin()
