#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import rospy
import sys

from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal


class CheckUseSimTime(object):
    def __init__(self):
        self.warning = rospy.get_param('~warning', True)
        self.override = rospy.get_param('~override', True)
        self.client_jp = actionlib.SimpleActionClient(
            '/robotsound_jp', SoundRequestAction)

    def run(self):
        use_sim_time = rospy.get_param('/use_sim_time', None)
        if use_sim_time:
            if self.warning:
                self._speak(
                    self.client_jp,
                    'use sim timeパラメータがTrueで設定されています',
                    'jp')
            if self.override:
                if self.warning:
                    self._speak(
                        self.client_jp,
                        'use sim timeパラメータをFalseに上書きます',
                        'jp')
                rospy.set_param('/use_sim_time', False)

    def _speak(self, client, speech_text, lang=None):
        client.wait_for_server(timeout=rospy.Duration(1.0))
        sound_goal = SoundRequestGoal()
        sound_goal.sound_request.sound = -3
        sound_goal.sound_request.command = 1
        sound_goal.sound_request.volume = 1.0
        if lang is not None:
            sound_goal.sound_request.arg2 = lang
        sound_goal.sound_request.arg = speech_text
        client.send_goal(sound_goal)
        client.wait_for_result()
        return client.get_result()


if __name__ == '__main__':
    rospy.init_node('check_use_sim_time')
    app = CheckUseSimTime()
    app.run()
    sys.exit(0)
