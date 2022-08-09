#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu


class ImuCorrector(object):
    def __init__(self):
        super(ImuCorrector, self).__init__()
        self.sub = rospy.Subscriber(
            '~input', Imu, self._cb, queue_size=1)
        self.pub = rospy.Publisher(
            '~output', Imu, queue_size=1)

    def _cb(self, msg):
        if msg.header.frame_id == '':
            msg.header.frame_id = 'base_link'
        if all([x == 0 for x in msg.angular_velocity_covariance]):
            msg.angular_velocity_covariance = [4e-6, 0, 0,
                                               0, 4e-6, 0,
                                               0, 0, 4e-6]
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('imu_corrector')
    app = ImuCorrector()
    rospy.spin()
