#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry


class OdomCorrector(object):
    def __init__(self):
        super(OdomCorrector, self).__init__()
        self.sub = rospy.Subscriber(
            '~input', Odometry, self._cb, queue_size=1)
        self.pub = rospy.Publisher(
            '~output', Odometry, queue_size=1)
        self.covariance = [1e-3, 0, 0, 0, 0, 0,
                           0, 1e-3, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 1e-2]

    def _cb(self, msg):
        msg.pose.covariance = self.covariance
        msg.twist.covariance = self.covariance
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('odom_corrector')
    app = OdomCorrector()
    rospy.spin()
