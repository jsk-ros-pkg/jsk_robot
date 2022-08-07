#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from jsk_recognition_msgs.msg import PeoplePoseArray
from dialogflow_task_executive.msg import DialogResponse
import time
import message_filters

from threading import Lock


class testNode():
    def __init__(self):
        # Publisher
        self.pub = rospy.Publisher('/emotion', String, queue_size=1)

        # Subscriber
        self.sub1 = rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, self.speech_callback)
        self.sub2 = rospy.Subscriber('/people_pose', PeoplePoseArray, callback=self.callback)
        self.sub3 = rospy.Subscriber('/dialog_response', DialogResponse, callback=self.df_cb)
        self.duration_time = rospy.Duration(30)
        self.prev_pose_detected_time = rospy.Time.now() - self.duration_time

        self.lock = Lock()

        queue_size = 10
        fps = 100.
        delay = 1 / fps * 0.5

    def callback(self, msg):
        if (rospy.Time.now() - self.prev_pose_detected_time) < self.duration_time:
            return
        poses = msg.poses
        if len(poses) >= 1:
            item = poses[0].limb_names
            scores = poses[0].scores
            target_limbs = [
                "left_eye", "nose", "right_eye", "right_ear",
                "left_ear"]
            if all([name in item for name in target_limbs]):
                message = "happy"
                rospy.loginfo("received {}, current emotion is {}".format(item, message))
                with self.lock:
                    self.publish(message)
                    self.prev_pose_detected_time = rospy.Time.now()
                    rospy.sleep(5.0)

    def publish(self, data):
        self.pub.publish(data)

    def speech_callback(self, msg):
        word = msg.transcript[0]
        if word in ["こんにちは",
                    "ヤッホー",
                    "こんばんは",
                    "おはよう",
                    "おはようございます"]:
            message = "happy"
        elif word in ["可愛いね",
                      "可愛い",
                      "かわいいね",
                      "かわいい"]:
            message = "joy"
        elif word in ["散歩に行こう",
                      "散歩",
                      "行こう",
                      "いこう"]:
            message = "affirmation"
        elif word in ["今日は行けないよ",
                      "いけないよ",
                      "行けないよ"]:
            message = "negation"
        elif word in ["大好きだよ",
                      "好き",
                      "すき"]:
            message = "love"
        elif word in ["あっち行って",
                      "さようなら"]:
            message = "scared"
        else:
            message = word
        rospy.loginfo("received {}, current emotion is {}".format(word, message))
        with self.lock:
            self.publish(message)
            rospy.sleep(5.0)

    def df_cb(self, data):
        if data.action == "Happy" or data.action == "input.welcome":
            self.publish("happy")
        elif data.action == "Smirking" or data.action == "Squinting":
            self.publish("joy")
        elif data.action == "Love":
            self.publish("love")
        elif data.action == "Fearful" or data.action == "Cry":
            self.publish("scared")
        elif data.action == "Relived":
            self.publish("affirmation")
        elif data.action == "Boring" or data.action == "Unpleasant":
            self.publish("negation")
        elif data.action == "input.unknown":
            self.publish("curious")
        elif data.action == "Angry" or data.action == "Astonished":
            self.publish("astonished")
        else:
            rospy.logwarn("Unknown emotion")

if __name__ == '__main__':
    rospy.init_node('speech_to_emotion')

    time.sleep(3.0)
    node = testNode()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
