#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from spot_msgs.msg import ManipulatorState
from sensor_msgs.msg import Joy

class EndEffectorToJoy(object):

    def __init__(self):
        self._sub = rospy.Subscriber(
            '/spot/status/manipulator_state',
            ManipulatorState,
            self._cb)
        self._pub = rospy.Publisher(
            '/joy_head/joy_raw',
            Joy)
        self._deadzone = float(rospy.get_param('~deadzone', 4.0))
        self._maxrange = int(rospy.get_param('~maxrange', 10.0))

    def _cb(self, msg):
        joy = Joy()
        joy.header.frame_id = rospy.get_name()
        joy.header.stamp = rospy.Time.now()
        x = msg.estimated_end_effector_force_in_hand.x
        y = msg.estimated_end_effector_force_in_hand.y
        z = msg.estimated_end_effector_force_in_hand.z
        rospy.loginfo_throttle(10, "end_effector_froce_in_hand = ({:4.1f}, {:4.1f}, {:4.1f}), stow_state = {}".format(x, y, z, msg.stow_state))
        if msg.stow_state == ManipulatorState.STOWSTATE_STOWED:
            x = 0
            y = 0
        else:
            sign_x = cmp(x, 0)
            sign_y = cmp(y, 0)
            x = abs(x)
            y = abs(y)
            if x > self._maxrange: x = self._maxrange
            if y > self._maxrange: y = self._maxrange
            x = 0 if x < self._deadzone else x - self._deadzone
            y = 0 if y < self._deadzone else y - self._deadzone
            x = sign_x * x / (self._maxrange - self._deadzone)
            y = sign_y * y / (self._maxrange - self._deadzone)
        joy.axes = [x, y, 0, 0, 0, 0]
        rospy.loginfo_throttle(10, "                                                               ({:4.1f}, {:4.1f}, {:4.1f})".format(x, y, 0))
        joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # dummy button data
        self._pub.publish(joy)

def main():
    rospy.init_node('end_effector_to_joy')
    end_effector_to_joy = EndEffectorToJoy()
    rospy.spin()


if __name__ == '__main__':
    main()
