#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy

class JoyMsgSwitcher():
    def __init__(self):
        self.counter = 1
        self.pre = 0
        self.angle_pub = rospy.Publisher('~angle_mode', Joy, queue_size=1)
        self.ik_pub = rospy.Publisher('~ik_mode', Joy, queue_size=1)
        rospy.Subscriber('joy', Joy, self.callback)

    def callback(self, msg):
        joy_msg = msg
        l3_button = msg.buttons[10]

        if l3_button - self.pre == 1:
            self.counter += 1

        if self.counter % 2 == 0:
            self.ik_pub.publish(joy_msg)
        else:
            self.angle_pub.publish(joy_msg)

        self.pre = l3_button

if __name__=='__main__':
    rospy.init_node('joy_msg_switcher')
    joy_msg_swither = JoyMsgSwitcher()
    rospy.spin()
