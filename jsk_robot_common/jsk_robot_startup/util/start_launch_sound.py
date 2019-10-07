#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
# topic --> /robotsound or /robotsound_jp

rospy.init_node("finish_launch_sound")
p = rospy.Publisher("/robotsound", SoundRequest, queue_size=1)

rospy.sleep(5)                  # sleep to wait for connection
msg = SoundRequest()
msg.sound = SoundRequest.SAY
msg.command = SoundRequest.PLAY_ONCE
if hasattr(msg, 'volume'):
    msg.volume = 1.0
msg.arg = "Launching"
p.publish(msg)
