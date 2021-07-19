#!/usr/bin/env python

# https://www.youtube.com/watch?v=HM-xG0qXaeA&&
# ffmpeg -i R2D2\ all\ Sounds\ -\ Star\ Wars\ free\ sounds.mp4 -ss 48 -t 10 R2D2.wav

import rospy
import time, socket, os
import netifaces as ni
import rospkg

from sound_play.libsoundplay import SoundClient
import actionlib
from sound_play.msg import SoundRequestAction

if __name__ == "__main__":
    rospy.init_node("boot_sound")
    if rospy.has_param("~wav_file"):
        wav_file = rospy.get_param("~wav_file")
    else:
        wav_file = "/usr/share/sounds/alsa/Front_Center.wav"

    sound = SoundClient()
    time.sleep(1) # ???
    ac = actionlib.SimpleActionClient('sound_play', SoundRequestAction)
    ac.wait_for_server()


    interfaces =  [x for x in ni.interfaces() if x[0:3] in ['eth', 'enp', 'wla', 'wlp'] and
                   2 in ni.ifaddresses(x).keys()]
    if len(interfaces) > 0 :
        ip = ni.ifaddresses(interfaces[0])[2][0]['addr']
    else:
        ip = None

    # play sound
    rospy.loginfo("Playing {}".format(wav_file))
    sound.playWave(wav_file)
    time.sleep(10) # make sure to topic is going out

    # notify ip address
    ip_text = "My internet address is {}".format(ip)
    rospy.loginfo(ip_text)
    ip_text = ip_text.replace('.', ', ')
    sound.say(ip_text)
    time.sleep(1) # make sure to topic is going out





