#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import time
from sensor_msgs.msg import Joy

class JoyTopicCompletion:

    def sigmoid(self, x):
        return 1 / (1 + math.exp(-x))

    def callback(self,msg):
        # check if there is a change
        for axesdata in msg.axes:
            if axesdata != 0.0:
                buttons = list(msg.buttons)
                buttons[self.enable_button] = 1
                msg.buttons = tuple(buttons)

        if not self.pass_through:
            # https://jglobal.jst.go.jp/detail?JGLOBAL_ID=200902183343247230
            # 引き綱型インタフェースによる自立型4脚移動ロボットの誘導
            # Leading a Self-Contained Quardruped Mobile Robot with a ”Dog-Walking” Interface.
            # 椛沢光隆 (東大) ,  安藤毅 (東大) ,  近野敦 (東大) ,  稲葉雅幸 (東大) ,  井上博允 (東大)
            #
            # rope interface
            # r : linear velocity
            #     K(F - F_thre) F >= F_thre
            #     0             F  < F_thre

            # a : linear vector
            #       0 (d - d_0) < d_thr
            #     180 (d - d_0) > 180 - d_thr

            # dw/dr : angular velocity
            #         K_dw/dr a            |a| <  a_thr
            #         (d/omega)/dr sin(a)  |a| >= a_thr

            # Increase the influence of joystick tilt
            # Inprove tracking in rope pulling direction
            # See https://github.com/k-okada/jsk_robot/pull/22
            x = msg.axes[self.axis_linear_x]
            y = msg.axes[self.axis_linear_y]
            v = math.exp(math.sqrt(x*x + y*y)) - 1
            r = math.atan2(y, x)

            yy = 0
            xx = v * math.cos(r)

            if v > 0:
                self.last_publish_time = rospy.Time.now()
                rr = (self.sigmoid(r) - 0.5) * 2 * math.pi
            else:
                rr = 0

            rospy.loginfo("raw: x={:5.2f}, y={:5.2f}, v={:5.2f}, r={:5.2f} -> cmd: x={:5.2f}, y={:5.2f}, r={:5.2f}".format(x, y, v, r, xx, yy, rr))

            msg.axes = list(msg.axes)  # tuple to list
            msg.axes[self.axis_linear_x] = max(min(xx, self.axis_linear_x_max), self.axis_linear_x_min)
            msg.axes[self.axis_linear_y] = max(min(yy, self.axis_linear_y_max), self.axis_linear_y_min)
            msg.axes[self.axis_angular] = max(min(rr, self.axis_angular_max), self.axis_angular_min)

        self.pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(10)
        msg = Joy()
        msg.axes = [0, 0, 0, 0, 0, 0]
        msg.buttons = [0, 0, 0, 0, 0, 0]
        while not rospy.is_shutdown():
            rate.sleep()
            if rospy.Time.now() - self.last_publish_time > rospy.Duration(3.0):
                self.pub.publish(msg)

    def get_param(self, key1, key2, value):
        p = rospy.get_param(key1)
        if type(p) == dict and p.has_key(key2):
            return p[key2]
        else:
            return value

    def __init__(self):
        rospy.init_node('joy_topic_completion')
        self.pass_through = rospy.get_param('~pass_through', True)
        self.enable_button = int(rospy.get_param('~enable_button', 0))
        self.axis_linear_x = int(rospy.get_param('~axis_linear', {'x': 1})['x'])
        self.axis_linear_y = int(rospy.get_param('~axis_linear', {'y': 2})['y'])
        self.axis_angular = int(rospy.get_param('~axis_angular', {'yaw': 0})['yaw'])
        self.axis_linear_x_min = float(self.get_param('~axis_linear', 'x_min',  -1))
        self.axis_linear_x_max = float(self.get_param('~axis_linear', 'x_max',   1))
        self.axis_linear_y_min = float(self.get_param('~axis_linear', 'y_min',  -1))
        self.axis_linear_y_max = float(self.get_param('~axis_linear', 'y_max',   1))
        self.axis_angular_min = float(self.get_param('~axis_angular', 'yaw_min', -0.5))
        self.axis_angular_max = float(self.get_param('~axis_angular', 'yaw_max',  0.5))

        self.last_publish_time = rospy.Time.now()

        # set attribute before start subscriber
        # queue_size=1 is important beacuse we want take only latest data
        self.sub = rospy.Subscriber("~joy_input", Joy, self.callback, queue_size=1)
        self.pub = rospy.Publisher('~joy_output', Joy, queue_size=1)

    def __del__(self):
        msg = Joy()
        msg.axes = [0]*(max(self.axis_linear_x, self.axis_linear_y, self.axis_angular)+1)
        msg.butons = [0]*(self.enable_button+1)
        self.pub.publish(msg)
        time.sleep(0.75)


if __name__ == '__main__':

    joy_topic_completion = JoyTopicCompletion()
    joy_topic_completion.spin()
    del(joy_topic_completion)
