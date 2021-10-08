#!/usr/bin/env python

PKG = 'speak_and_wait_recovery'
NAME = 'speak_and_wait_recovery_test'

import rostest
import unittest
import sys

import actionlib
import rospy
from sound_play.msg import SoundRequestAction, SoundRequestResult


class TestSpeakAndWaitRecovery(unittest.TestCase):

    def handler(self, goal):

        self.msg = goal.sound_request
        self.action_server.set_succeeded(SoundRequestResult(playing=True,stamp=rospy.Time.now()))

    def test_speak_and_wait_recovery(self):

        self.msg = None
        self.action_server = actionlib.SimpleActionServer( '/sound_play', SoundRequestAction, execute_cb=self.handler, auto_start=False)
        self.action_server.start()
        while not rospy.is_shutdown():
            if self.msg is not None:
                msg = self.msg
                break

        self.assertEqual(msg.sound, -3)
        self.assertEqual(msg.command, 1)
        self.assertEqual(msg.volume, 1.0)
        self.assertEqual(msg.arg, 'test')


if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestSpeakAndWaitRecovery, sys.argv)
