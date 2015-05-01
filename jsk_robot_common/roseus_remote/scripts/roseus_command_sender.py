#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import rospy
from roseus_remote.msg import RawCommand


def cmd_res_cb(msg):
    input_str = str(msg.data).rstrip(' \t\r\n\0')
    print input_str

def on_shutdown():
    rospy.logwarn("to exit immediately, press Enter")

def main():
    global cmd_res
    rospy.init_node("roseus_remote_command_sender", anonymous=True)
    rospy.on_shutdown(on_shutdown)
    pub = rospy.Publisher("output", RawCommand)
    sub = rospy.Subscriber("input", RawCommand, cmd_res_cb)
    while not rospy.is_shutdown():
        try:
            cmd = raw_input("roseus$ ")
            if cmd:
                pub.publish(RawCommand(data=cmd))
                rospy.sleep(0.01)
        except Exception as e:
            rospy.logwarn("caught exception: " + str(e))
            rospy.sleep(1.0)

if __name__ == '__main__':
    main()
