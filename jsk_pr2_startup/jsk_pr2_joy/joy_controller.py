#!/usr/bin/env python

import os

import rospy
import roslib

roslib.load_manifest("jsk_pr2_startup")

from sensor_msgs.msg import Joy

class JoyController():
    def __init__(self):
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.previous_joy = None
        rospy.spin()

    def joy_callback(self, joy):
        self.joy = joy
        self.joy_execute()
        self.previous_joy = joy

    def joy_execute(self):
        if self.check_pushed(0):
            print "0 is pushed"
        pass

    def check_pushed(self, index):
        if self.previous_joy != None and index < len(self.joy.buttons):
            if self.previous_joy.buttons[index] == 0 and self.joy.buttons[index] == 1:
                return True
        return False

if __name__ =='__main__':
    rospy.init_node('joy_controller', anonymous=True)
    joy_controller = JoyController()
