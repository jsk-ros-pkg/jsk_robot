#!/usr/bin/env python

import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


if __name__ == '__main__':

    rospy.init_node('print_speech_to_text')

    def callback(msg):
        if len(msg.transcript) > 0:
            rospy.loginfo('{}'.format(msg.transcript[0]))

    sub = rospy.Subscriber(
        '/speech_to_text_jp',
        SpeechRecognitionCandidates,
        callback)

    rospy.spin()
