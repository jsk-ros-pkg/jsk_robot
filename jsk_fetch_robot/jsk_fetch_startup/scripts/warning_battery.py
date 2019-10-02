# -*- coding: utf-8 -*-

import actionlib
import rospy
from time import sleep
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequest, SoundRequestGoal
from power_msgs.msg import BatteryState

class BatteryWarning:
    def __init__(self, threshould):
        self.speak_client = actionlib.SimpleActionClient("/robotsound_jp", SoundRequestAction)
        self.charge_level_percentage = 100.0
        self.threshould = threshould 
        self.subscriber = rospy.Subscriber("/battery_state", BatteryState, self.callback)

    def _speak(self, sentence):
        req = SoundRequest()
        req.command = SoundRequest.PLAY_ONCE
        req.sound = SoundRequest.SAY
        req.arg = sentence
        req.arg2 = "ja"
        req.volume = 1.0
        self.speak_client.send_goal(SoundRequestGoal(sound_request=req))
        self.speak_client.wait_for_result(timeout=rospy.Duration(10))

    def callback(self, msg):
        self.charge_level = msg.charge_level * 100
        if self.charge_level < self.threshould:
            print msg.charge_level * 100
            sentence = "バッテリー残り" + str(int(self.charge_level)) + "パーセント"  
            self._speak(sentence)
            sleep(0.2)
            self._speak("もう限界ですので、僕をお家にかえしてください")

if __name__ == '__main__':
    rospy.init_node("battery_warning")
    charge_level_threshould = 80.0 # percent
    bw = BatteryWarning(charge_level_threshould)
    rospy.spin()

