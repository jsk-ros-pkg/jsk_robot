#!/usr/bin/env python

PKG = 'speak_and_wait_recovery'
NAME = 'speak_and_wait_recovery_test'

import rostest
import unittest
import sys
import threading

import actionlib
import rospy
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestResult
from sound_play.msg import SoundRequest


class TestSpeakAndWaitRecovery(unittest.TestCase):

    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node(NAME)
        self.lock_for_msg = threading.Lock()
        self.msg = SoundRequest()
        self.action_server = \
                actionlib.SimpleActionServer( '/sound_play', SoundRequestAction, execute_cb=self.handler, auto_start=False)
        self.action_server.start()

    def handler(self, goal):

        with self.lock_for_msg:
            self.msg = goal.sound_request
        rospy.logwarn('Receive ad message')
        self.action_server.set_succeeded(SoundRequestResult(playing=True,stamp=rospy.Time.now()))

    def test_speak_and_wait_recovery(self):

        rospy.init_node(NAME)
        msg = SoundRequest()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            with self.lock_for_msg:
                if self.msg.arg != '':
                    msg = self.msg
                    break
                else:
                    rospy.logwarn('waiting')
            rate.sleep()

        rospy.logwarn('msg: {}'.format(msg))
        self.assertEqual(msg.sound, -3)
        self.assertEqual(msg.command, 1)
        self.assertEqual(msg.volume, 1.0)
        self.assertEqual(msg.arg, 'test')


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestSpeakAndWaitRecovery, sys.argv)
