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
        self.speak_client = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.duration = rospy.get_param('~duration', 180.0)
        self.threshold = rospy.get_param('~charge_level_threshold', 40.0)
        self.step = rospy.get_param('~charge_level_step', 10.0)
        self.subscriber = rospy.Subscriber(
            '/battery_state', BatteryState, self._cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.duration), self._timer_cb)
        self.charge_level = None
        self.prev_charge_level = None
        self.is_charging = False

    def _speak(self, sentence):
        req = SoundRequest()
        req.command = SoundRequest.PLAY_ONCE
        req.sound = SoundRequest.SAY
        req.arg = sentence
        req.arg2 = 'ja'
        req.volume = 0.8
        self.speak_client.send_goal(SoundRequestGoal(sound_request=req))
        self.speak_client.wait_for_result(timeout=rospy.Duration(10))

    def _warn(self):
        if self.charge_level < self.threshold and not self.is_charging:
            rospy.logerr("Low battery: only {}% remaining".format(self.charge_level))
            sentence = "バッテリー残り{}パーセントです。".format(self.charge_level)
            sentence += "もう限界ですので、僕をお家にかえしてください。"
            self._speak(sentence)
            self.prev_charge_level = self.charge_level
        elif (self.prev_charge_level // self.step) > (self.charge_level // self.step):
            rospy.loginfo("Battery: {}% remaining".format(self.charge_level))
            sentence = "バッテリー残り{}パーセントです。".format(self.charge_level)
            self._speak(sentence)
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
