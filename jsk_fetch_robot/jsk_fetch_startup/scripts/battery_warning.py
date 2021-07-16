#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import rospy

from power_msgs.msg import BatteryState
from sound_play.msg import SoundRequest
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal


class BatteryWarning(object):
    def __init__(self):
        self.client_en = actionlib.SimpleActionClient(
            '/sound_play', SoundRequestAction)
        self.client_jp = actionlib.SimpleActionClient(
            '/robotsound_jp', SoundRequestAction)
        self.duration = rospy.get_param('~duration', 180.0)
        self.threshold = rospy.get_param('~charge_level_threshold', 40.0)
        self.step = rospy.get_param('~charge_level_step', 10.0)
        self.subscriber = rospy.Subscriber(
            '/battery_state', BatteryState, self._cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.duration), self._timer_cb)
        self.charge_level = None
        self.prev_charge_level = None
        self.is_charging = False

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

    def _warn(self):
        if self.charge_level < self.threshold and not self.is_charging:
            rospy.logerr("Low battery: only {}% remaining".format(self.charge_level))
            sentence_jp = "バッテリー残り{}パーセントです。".format(self.charge_level)
            sentence_jp += "もう限界ですので、僕をお家にかえしてください。"
            sentence_en = "My battery is {} percent remaining.".format(self.charge_level)
            sentence_en += "I want to go back home to charge my battery."
            self._speak(self.client_jp, sentence_jp, 'jp')
            self._speak(self.client_en, sentence_en)
            self.prev_charge_level = self.charge_level
        elif (self.prev_charge_level // self.step) > (self.charge_level // self.step):
            rospy.loginfo("Battery: {}% remaining".format(self.charge_level))
            sentence_jp = "バッテリー残り{}パーセントです。".format(self.charge_level)
            sentence_en = "My battery is {} percent remaining.".format(self.charge_level)
            self._speak(self.client_jp, sentence_jp, 'jp')
            self._speak(self.client_en, sentence_en)
            self.prev_charge_level = self.charge_level

    def _cb(self, msg):
        is_first_time = self.charge_level is None
        self.charge_level = int(msg.charge_level * 100)
        self.is_charging = msg.is_charging
        if is_first_time:
            self.prev_charge_level = self.charge_level + self.step
            self._warn()

    def _timer_cb(self, event):
        self._warn()

if __name__ == '__main__':
    rospy.init_node('battery_warning')
    bw = BatteryWarning()
    rospy.spin()
