#!/usr/bin/env python

import rospy
import numpy
from std_msgs.msg import Empty as EmptyMsg
import tf
import time

def callback(msg):
    global position
    global orientation

    rospy.loginfo("reset odom_ground")
    try:
        stamp = rospy.Time(0)
        (l_trans,l_rot) = tfl.lookupTransform(odom_frame, "lleg_end_coords", stamp)
        (r_trans,r_rot) = tfl.lookupTransform(odom_frame, "rleg_end_coords", stamp)
        position = numpy.mean([l_trans, r_trans], axis=0)
        orientation = [0, 0, 0, 1]
    except:
        rospy.logerr("Failed to reset {} frame".format(odom_ground_frame))
        return

if __name__ == "__main__":
    rospy.init_node("odom_ground_publisher")
    sub = rospy.Subscriber("/odom_init_trigger", EmptyMsg, callback)
    tfl = tf.TransformListener(True, rospy.Duration(10))
    tfb = tf.TransformBroadcaster()
    odom_frame = rospy.get_param("~odom_frame", "odom")
    rospy.loginfo("odom frame: {}".format(odom_frame))
    odom_ground_frame = rospy.get_param("~odom_ground_frame", "odom_ground")
    rospy.loginfo("odom ground frame: {}".format(odom_ground_frame))
    rate = rospy.get_param("~rate", 500)
    r = rospy.Rate(rate)

    position = None
    orientation = None

    while not rospy.is_shutdown():
        if position is not None:
            try:
                tfb.sendTransform(position, orientation, rospy.Time.now(), odom_ground_frame, odom_frame)
            except:
                rospy.logerr("Failed to broadcast {} to {} transform".format(odom_frame, odom_ground_frame))
        else:
            rospy.loginfo("make {} frame".format(odom_ground_frame))
            dummy_msg = EmptyMsg()
            callback(dummy_msg)
            time.sleep(1) # wait to make odom ground tf
        r.sleep()
