#! /usr/bin/env python
# -*- coding: utf-8 -*-

'''
start/ kill play audio stream node by using subprocess
'''

import rospy
from subprocess import *
from std_srvs.srv import (
    EmptyResponse,
    Empty)

def handle_start_srv(req):
    try:
        p=Popen(['roslaunch', 'jsk_pepper_startup', 'play_audio_stream.launch']) 
        return EmptyResponse()
    except RuntimeError, e:
        rospy.logerr("Exception caught:\n%s", e)
        return res

def handle_stop_srv(req):
    try:
        p2=Popen(['rosnode','list'], stdout=PIPE)
        p2.wait()
        nodelist_tmp=p2.communicate()
        # nodelist_tmp: ('/pepper_1556630474468299832\n/rosout\n', None)
        nodelist=nodelist_tmp[0]
        # nodelist: '/pepper_1556630474468299832\n/rosout\n'
        nodelist=nodelist.split("\n")
        # nodelist: ['/pepper_1556630474468299832', '/rosout', '']
        for i in range(len(nodelist)):
            if nodelist[i] == '/play_audio_stream_node':
                call(['rosnode', 'kill', nodelist[i]])
                break
        return EmptyResponse()
    except RuntimeError, e:
        rospy.logerr("Exception caught:\n%s", e)
        return res                    
        
if __name__=="__main__":
    rospy.init_node("play_audio_stream")
    start_srv = rospy.Service('start_subscribe_audio_topic', Empty, handle_start_srv)
    stop_srv = rospy.Service('stop_subscribe_audio_topic', Empty, handle_stop_srv)
    rospy.spin()
