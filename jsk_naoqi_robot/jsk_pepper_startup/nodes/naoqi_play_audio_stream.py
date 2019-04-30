#! /usr/bin/env python
# -*- coding: utf-8 -*-

'''
Play audio stream by using PyAudio from audio topic of Naoqi robot
referred to this discussion in ROS Sig Aldebaran: https://groups.google.com/forum/#!topic/ros-sig-aldebaran/M7Q3P51Akv8
'''

import rospy
import pyaudio
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi_bridge_msgs.msg import AudioBuffer
from std_srvs.srv import (
    EmptyResponse,
    Empty)

class PlayAudioStream(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'play_audio_stream')
        # instantiate PyAudio
        self.p = pyaudio.PyAudio()

        # open stream
        self.stream = self.p.open(format=pyaudio.paInt16,
                                  # reference https://github.com/ros-naoqi/naoqi_driver/blob/master/src/event/audio.cpp#L168
                                  channels=4,
                                  # reference https://github.com/ros-naoqi/naoqi_driver/blob/master/src/event/audio.cpp#L167
                                  rate=48000,
                                  output=True)            
        # start the stream
        self.stream.start_stream()

        # create audio topic subscriber
        self.sub = rospy.Subscriber('~audio', AudioBuffer, self.topic_cb)

        # create service handler
        # to start play audio stream, please call 'start_subscribe_audio_topic' service at first 
        self.enable_publish_topic = False
        self.start_srv = rospy.Service('start_subscribe_audio_topic', Empty, self.handle_start_srv)
        self.stop_srv = rospy.Service('stop_subscribe_audio_topic', Empty, self.handle_stop_srv)
        
        rospy.loginfo("play_audio_stream initialized")

    def topic_cb(self, msg):
        if self.enable_publish_topic:
            tmp = list(msg.data)
            dataBuff = ""            
            for i in range (0,len(tmp)) :
                # reference: https://github.com/ros-naoqi/naoqi_bridge/blob/master/naoqi_sensors_py/src/naoqi_sensors/naoqi_microphone.py#L116-L117
                if tmp[i]<0 :
                    tmp[i]=tmp[i]+65536
                # reference: https://github.com/ros-naoqi/naoqi_bridge/blob/master/naoqi_sensors_py/src/naoqi_sensors/naoqi_microphone.py#L111-L112
                dataBuff = dataBuff + chr(tmp[i]%256)
                dataBuff = dataBuff + chr( (tmp[i] - (tmp[i]%256)) /256)
                    
            self.stream.write(dataBuff)

    def handle_start_srv(self, req):
        try:
            self.enable_publish_topic = True
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handle_stop_srv(self, req):
        try:
            self.enable_publish_topic = False
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
                    
    def stop_audio_stream(self):
        # stop stream
        self.stream.stop_stream()
        self.stream.close()
        # close PyAudio
        self.p.terminate()                                                    
        
if __name__=="__main__":
    try:
        PlayAudioStream = PlayAudioStream()
        rospy.spin()
    except KeyboardInterrupt:
        self.stop_audio_stream()        
