#!/usr/bin/env python

import sys
import argparse

import rospy
import rosbag
import tf2_ros

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

def main():

    rospy.init_node( 'static_tf_republisher' )

    topicname = '/tf_static'

    myargv = rospy.myargv(argv=sys.argv)
    if len(myargv) > 1:
        bagfilename = myargv[1]
    else:
        bagfilename = rospy.get_param("~file")

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    list_messages = []
    transforms = []
    with rosbag.Bag( bagfilename, 'r' ) as inputbag:
        for topic, msg, t in inputbag.read_messages( '/tf_static' ):
            transforms.extend( msg.transforms )

    rospy.loginfo('republish /tf_static with {} TransformStamped messages'.format(len(transforms)))

    broadcaster.sendTransform(transforms)
    rospy.spin()

if __name__=='__main__':
    main()
