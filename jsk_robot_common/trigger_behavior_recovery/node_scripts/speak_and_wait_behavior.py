#!/usr/bin/env python

import rospy
import actionlib
from sound_play.libsoundplay import SoundClient
from trigger_behavior_recovery.msg import


class SpeakAndWaitBehavior(object):

    def __init__(self):

        self._speak_text = rospy.get_param('~speak_text', 'Hello World!')
        self._duraiton_wait = rospy.get_param('~duration_wait', 5.0)
        self._sound_action_name = rospy.get_param('~sound_action', 'sound_play')

        self._sound_client = SoundClient(blocking=True, sound_action = self._sound_action_name)

        self._action_server = 

    def handler(self, goal):

        self._sound_client.say(self._speak_text)
        rospy.sleep(self._duration_wait)
