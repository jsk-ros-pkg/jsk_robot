#!/usr/bin/env python
# -*- coding: utf-8 -*-

import requests
from pathlib import Path
from threading import Lock

import actionlib
import actionlib_msgs.msg
import rospy
from sound_play.msg import SoundRequest
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


_sound_play_clients = {}


def play_sound(sound,
               lang='',
               topic_name='robotsound',
               volume=1.0,
               wait=False):
    """
    Plays sound using sound_play server
    Args:
        sound: if sound is pathname, plays sound file located at given path
            if it is number, server plays builtin sound
            otherwise server plays sound as speech sentence
        topic-name: namespace of sound_play server
        wait: wait until sound is played
    """
    msg = SoundRequest(command=SoundRequest.PLAY_ONCE)
    if isinstance(sound, int):
        msg.sound = sound
    elif isinstance(sound, str) and Path(sound).exists():
        msg.sound = SoundRequest.PLAY_FILE
        msg.arg = sound
    elif isinstance(sound, str):
        msg.sound = SoundRequest.SAY
        msg.arg = sound
    else:
        raise ValueError

    if hasattr(msg, 'volume'):
        msg.volume = volume

    if topic_name in _sound_play_clients:
        client = _sound_play_clients[topic_name]
    else:
        client = actionlib.SimpleActionClient(
            topic_name,
            SoundRequestAction)
    client.wait_for_server()

    goal = SoundRequestGoal()
    if client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE:
        client.cancel_goal()
        client.wait_for_result(timeout=rospy.Duration(10))
    goal.sound_request = msg
    _sound_play_clients[topic_name] = client
    client.send_goal(goal)

    if wait is True:
        client.wait_for_result(timeout=rospy.Duration(10))
    return client


def speak_en(text,
             topic_name='robotsound',
             wait=False):
    """Speak english sentence

    """
    return play_sound(text,
                      topic_name=topic_name,
                      wait=wait)


def speak_jp(text,
             topic_name='robotsound_jp',
             wait=False):
    """Speak japanese sentence

    """
    return play_sound(text,
                      lang='ja',
                      topic_name=topic_name,
                      wait=wait)


class InteractiveChatNode(object):

    def __init__(self):
        self.current_msg = None
        self.lock = Lock()
        self.sub = rospy.Subscriber('/Tablet/voice',
                                    SpeechRecognitionCandidates,
                                    callback=self.callback,
                                    queue_size=1)

    def callback(self, msg):
        with self.lock:
            self.current_msg = msg

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_msg is not None:
                with self.lock:
                    voice_data = self.current_msg.transcript[0]
                    rospy.loginfo('recived: {}'.format(
                        self.current_msg.transcript[0]))
                    if 'こんにちは' in voice_data:
                        speak_jp('こんにちは、エアロです')
                    self.current_msg = None
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('interactive_chat_node')
    node = InteractiveChatNode()
    node.run()
