#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from geometry_msgs.msg import PoseStamped


def callback(msg):
    rospy.loginfo(msg.pose.position.x)


def main():
    rospy.init_node("lazy_subscriber")
    sub = None
    i = 0
    r = rospy.Rate(1.0 / 5)
    while not rospy.is_shutdown():
        r.sleep()
        if sub is None:
            sub = rospy.Subscriber(
                "~input", PoseStamped, callback)
            rospy.loginfo("started subscribe")
        else:
            sub.unregister()
            sub = None
            rospy.loginfo("stopped subscribe")


if __name__ == '__main__':
    main()
