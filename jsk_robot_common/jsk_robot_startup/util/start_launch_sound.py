#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
# topic --> /robotsound or /robotsound_jp

rospy.init_node("finish_launch_sound")
p = rospy.Publisher("/robotsound", SoundRequest)

rospy.sleep(5)                  # sleep to wait for connection
msg = SoundRequest()
msg.sound = SoundRequest.SAY
msg.command = SoundRequest.PLAY_START
msg.arg = "Launching"
p.publish(msg)
