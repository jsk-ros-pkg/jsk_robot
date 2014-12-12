#!/usr/bin/env python

import os

import rospy
import roslib
from joy_controller import JoyController
import std_msgs

roslib.load_manifest("jsk_pr2_startup")

class ImageSnapshotJoy(JoyController):
    def __init__(self):
        self._head_pub = rospy.Publisher('/head_snap/snapshot', std_msgs.msg.Empty)
        self._lhand_pub = rospy.Publisher('/lhand_snap/snapshot', std_msgs.msg.Empty)
        self._rhand_pub = rospy.Publisher('/rhand_snap/snapshot', std_msgs.msg.Empty)
        self._head_pub_button = rospy.get_param('~head_pub_button', 0)
        self._lhand_pub_button = rospy.get_param('~lhand_pub_button', 1)
        self._rhand_pub_button = rospy.get_param('~rhand_pub_button', 2)
        JoyController.__init__(self)

    def joy_execute(self):
        if self.check_pushed(self._head_pub_button):
            self._head_pub.publish(std_msgs.msg.Empty())
        if self.check_pushed(self._lhand_pub_button):
            self._lhand_pub.publish(std_msgs.msg.Empty())
        if self.check_pushed(self._rhand_pub_button):
            self._rhand_pub.publish(std_msgs.msg.Empty())

if __name__ =='__main__':
    rospy.init_node('image_snapshot_joy', anonymous=True)
    image_snapshot_joy = ImageSnapshotJoy()

