#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import numpy as np
import rospy
from jsk_topic_tools import ConnectionBasedTransport
from geometry_msgs.msg import PoseStamped


class LazyPublisher(ConnectionBasedTransport):
    def __init__(self):
        super(LazyPublisher, self).__init__()
        self.pub = self.advertise(
            "~output", PoseStamped, queue_size=1)
        self.timer = None
        self.i = 0

    def subscribe(self):
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_cb)

    def unsubscribe(self):
        self.timer.shutdown()
        self.timer = None

    def timer_cb(self, event):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = np.sin(np.radians(self.i * 30))
        self.pub.publish(msg)
        self.i += 1


if __name__ == '__main__':
    rospy.init_node("lazy_publisher")
    p = LazyPublisher()
    rospy.spin()
