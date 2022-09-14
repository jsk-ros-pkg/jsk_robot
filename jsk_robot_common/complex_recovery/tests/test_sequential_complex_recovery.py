#!/usr/bin/env python

from sound_play.msg import SoundRequestResult
from sound_play.msg import SoundRequestAction
import rospy
import actionlib
import threading
import sys
import unittest
import rostest

PKG = 'complex_recovery'
NAME = 'sequential_complex_recovery_test'


class TestSequentialComplexRecovery(unittest.TestCase):

    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node(NAME)

        self.lock_for_msg = threading.Lock()
        self.speech_text_list = []

        self.action_server = \
            actionlib.SimpleActionServer(
                '/sound_play', SoundRequestAction, execute_cb=self.handler, auto_start=False)
        self.action_server.start()

    def handler(self, goal):

        with self.lock_for_msg:
            self.speech_text_list.append(goal.sound_request.arg)
        rospy.logwarn('Receive a message')
        self.action_server.set_succeeded(
            SoundRequestResult(playing=True, stamp=rospy.Time.now()))

    def test_sequential_complex_recovery(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            with self.lock_for_msg:
                if 'test0' in self.speech_text_list and 'test1' in self.speech_text_list:
                    break
                else:
                    rospy.logwarn('waiting')
            rate.sleep()

        with self.lock_for_msg:
            rospy.logwarn('speech_text_list: {}'.format(self.speech_text_list))
            self.assertTrue('test0' in self.speech_text_list and 'test1' in self.speech_text_list)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestSequentialComplexRecovery, sys.argv)
