#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import tf2_ros
from spot_msgs.msg import ManipulatorState
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, SetBoolResponse

# python3 does noth have cmp...
def cmp(a, b):
    return (a > b) - (a < b)

class EndEffectorToJoy(object):

    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)
        self._pub = rospy.Publisher(
            '/joy_head/joy_raw',
            Joy)
        self._sub = None
        self._enabled = rospy.Service('~set_enabled', SetBool, self._set_enabled)

        self._deadzone_x = float(rospy.get_param('~deadzone_x', 0.01))
        self._deadzone_y = float(rospy.get_param('~deadzone_y', 0.01))
        self._maxrange_y = float(rospy.get_param('~maxrange_x', 0.2))
        self._maxrange_x = float(rospy.get_param('~maxrange_y', 0.2))
        self._offset_x = float(rospy.get_param('~offset_x', 0.0))
        self._offset_y = float(rospy.get_param('~offset_y', 0.0))

    def _set_enabled(self, req):
        if req.data:
            self._center_x = float(rospy.get_param('~center_x', 0.576))
            self._center_y = float(rospy.get_param('~center_0', 0))
            rospy.logwarn("Enabled end_effector_to_joy with center ({}, {})".format(self._center_x, self._center_y))
            self._sub = rospy.Subscriber('/spot/status/manipulator_state', ManipulatorState, self._cb)
        else:
            rospy.logwarn("Disabled end_effector_to_joy with center")
            if self._sub:
                self._sub.unregister()
                self._sub = None
        return SetBoolResponse(True, 'Success')

    def _cb(self, msg):
        joy = Joy()
        joy.header.frame_id = rospy.get_name()
        joy.header.stamp = rospy.Time.now()
        x = msg.estimated_end_effector_force_in_hand.x
        y = msg.estimated_end_effector_force_in_hand.y
        z = msg.estimated_end_effector_force_in_hand.z
        try:
            trans = self._tf_buffer.lookup_transform('body', 'hand', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return None
        x = trans.transform.translation.x - self._center_x
        y = trans.transform.translation.y - self._center_y
        rospy.loginfo_throttle(10, "end_effector_froce_in_hand = ({:4.1f}, {:4.1f}, {:4.1f}), stow_state = {}".format(x, y, z, msg.stow_state))
        if msg.stow_state == ManipulatorState.STOWSTATE_STOWED:
            x = 0
            y = 0
        else:
            sign_x = cmp(x, 0)
            sign_y = cmp(y, 0)
            x = abs(x) + self._offset_x
            y = abs(y) + self._offset_y
            if x > self._maxrange_x: x = self._maxrange_x
            if y > self._maxrange_y: y = self._maxrange_y
            x = 0 if x < self._deadzone_x else x - self._deadzone_x
            y = 0 if y < self._deadzone_y else y - self._deadzone_y
            x = sign_x * x / (self._maxrange_x - self._deadzone_x)
            y = sign_y * y / (self._maxrange_y - self._deadzone_y)
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
