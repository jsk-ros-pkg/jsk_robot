# -*- coding: utf-8 -*-

import actionlib
import rospy
from time import sleep

from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequest, SoundRequestGoal
from power_msgs.msg import BatteryState

rospy.init_node("battery_warning")

speak_client = actionlib.SimpleActionClient("/robotsound_jp", SoundRequestAction)
waitEnough = speak_client.wait_for_server(rospy.Duration(10))

def speak(sentence):
    req = SoundRequest()
    req.command = SoundRequest.PLAY_ONCE
    req.sound = SoundRequest.SAY
    req.arg = sentence
    req.arg2 = "ja"
    req.volume = 1.0
    speak_client.send_goal(SoundRequestGoal(sound_request=req))
    speak_client.wait_for_result(timeout=rospy.Duration(10))

global charge_level
charge_level = None
def callback(msg):
    global charge_level
    charge_level = int(msg.charge_level * 100)
    if charge_level < 70:
        sentence = "バッテリー残り" + str(charge_level) + "パーセント"
        speak(sentence)
        sleep(0.2)
        speak("僕をお家にかえして")
        sleep(0.2)

rospy.Subscriber("/battery_state", BatteryState, callback)
rospy.spin()

