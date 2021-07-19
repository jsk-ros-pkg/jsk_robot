#!/usr/bin/env python

# https://www.youtube.com/watch?v=HM-xG0qXaeA&&
# ffmpeg -i R2D2\ all\ Sounds\ -\ Star\ Wars\ free\ sounds.mp4 -ss 48 -t 10 R2D2.wav

import netifaces as ni
import rospy

from sound_play.libsoundplay import SoundClient

if __name__ == "__main__":
    rospy.init_node("boot_sound")
    if rospy.has_param("~wav_file"):
        wav_file = rospy.get_param("~wav_file")
    else:
        wav_file = "/usr/share/sounds/alsa/Front_Center.wav"

    sound = SoundClient(sound_action='sound_play', blocking=True)
    sound.actionclient.wait_for_server()

    if len(ni.ifaddresses('eth0')) > 2:
        ip = ni.ifaddresses('eth0')[2][0]['addr']
    elif len(ni.ifaddresses('wlan0')) > 2:
        ip = ni.ifaddresses('wlan0')[2][0]['addr']
    else:
        ip = None

    # play sound
    rospy.loginfo("Playing {}".format(wav_file))
    sound.playWave(wav_file, replace=False)

    # notify ip address
    ip_text = "My internet address is {}".format(ip)
    rospy.loginfo(ip_text)
    ip_text = ip_text.replace('.', ', ')
    sound.say(ip_text, replace=False)
