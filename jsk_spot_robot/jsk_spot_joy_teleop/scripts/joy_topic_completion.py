#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
 
class JoyTopicCompletion:

    def callback(self,msg):

        # check if there is a change
        for axesdata in msg.axes:
            if axesdata != 0.0:
                buttons = list(msg.buttons)
                buttons[self.enable_button] = 1
                msg.buttons = tuple(buttons)

        self.pub.publish(msg)

    def __init__(self):

        rospy.init_node('joy_topic_completion')
        self.sub = rospy.Subscriber("~joy_input", Joy, self.callback)
        self.pub = rospy.Publisher('~joy_output', Joy, queue_size=10)
        self.enable_button = int(rospy.get_param('~enable_button', 0))


if __name__ == '__main__':

    joy_topic_completion = JoyTopicCompletion()
    rospy.spin()
