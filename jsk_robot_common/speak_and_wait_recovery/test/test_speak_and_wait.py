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
        self.action_server.set_succeeded(SoundRequestResult(playing=False,stamp=rospy.Time.now()))

    def test_speak_and_wait_recovery(self):

        rospy.init_node('test')
        self.msg = None
        self.action_server = actionlib.SimpleActionServer( '/sound_play', SoundRequestAction, execute_cb=self.handler, auto_start=False)
        self.action_server.start()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.msg is not None:
                msg = self.msg
                if msg.sound == -3 \
                    and msg.command == 1 \
                    and msg.volume == 1.0:
                    #and msg.arg == 'test':
                    self.assertTrue(True)
                else:
                    rospy.logerr('Invalid message recieved: {}'.format(msg))


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestSpeakAndWaitRecovery, sys.argv)
