#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionResult
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal


def goal_status(status):
    return {GoalStatus.PENDING: 'PENDING',
            GoalStatus.ACTIVE: 'ACTIVE',
            GoalStatus.PREEMPTED: 'PREEMPTED',
            GoalStatus.SUCCEEDED: 'SUCCEEDED',
            GoalStatus.ABORTED: 'ABORTED',
            GoalStatus.REJECTED: 'REJECTED',
            GoalStatus.PREEMPTING: 'PREEMPTING',
            GoalStatus.RECALLING: 'RECALLING',
            GoalStatus.RECALLED: 'RECALLED',
            GoalStatus.LOST: 'LOST'}[status]


class NavSpeak(object):
    def __init__(self):
        self.move_base_goal_sub = rospy.Subscriber(
            "/move_base/goal", MoveBaseActionGoal,
            self.move_base_goal_callback, queue_size=1)
        self.move_base_result_sub = rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult,
            self.move_base_result_callback, queue_size=1)
        self.robotsound_jp_status_sub = rospy.Subscriber(
            "/robotsound_jp/status", GoalStatusArray,
            self.robotsound_jp_status_callback, queue_size=1)
        self.sound = SoundClient(blocking=True)
        self.lang = rospy.get_param("~lang", "japanese")
        self.volume = rospy.get_param("~volume", 1.0)
        self.client = actionlib.SimpleActionClient(
            'robotsound_jp', SoundRequestAction)
        self.client.wait_for_server()
        self.is_speaking = False

    def move_base_goal_callback(self, msg):
        self.sound.play(2, volume=self.volume)

    def move_base_result_callback(self, msg):
        # Wait if other node is speaking
        while self.is_speaking is True:
            time.sleep(1)
        text = "{}: {}".format(goal_status(msg.status.status), msg.status.text)
        rospy.loginfo(text)
        if self.lang == "japanese":  # speak japanese
            sound_goal = SoundRequestGoal()
            sound_goal.sound_request.sound = -3
            sound_goal.sound_request.command = 1
            sound_goal.sound_request.volume = self.volume
            sound_goal.sound_request.arg2 = "jp"
        if msg.status.status == GoalStatus.SUCCEEDED:
            self.sound.play(1, volume=self.volume)
            time.sleep(1)
            if self.lang == "japanese":
                sound_goal.sound_request.arg = "到着しました"
                self.client.send_goal(sound_goal)
            else:
                self.sound.say(text, volume=self.volume)
        elif msg.status.status == GoalStatus.PREEMPTED:
            self.sound.play(2, volume=self.volume)
            time.sleep(1)
            if self.lang == "japanese":
                sound_goal.sound_request.arg = "別のゴールがセットされました"
                self.client.send_goal(sound_goal)
            else:
                self.sound.say(text, volume=self.volume)
        elif msg.status.status == GoalStatus.ABORTED:
            self.sound.play(3, volume=self.volume)
            time.sleep(1)
            if self.lang == "japanese":
                sound_goal.sound_request.arg = "中断しました"
                self.client.send_goal(sound_goal)
            else:
                self.sound.say(text, volume=self.volume)
        else:
            self.sound.play(4, volume=self.volume)
            time.sleep(1)
            self.sound.say(text, volume=self.volume)

    def robotsound_jp_status_callback(self, msg):
        if len(msg.status_list) == 0:
            self.is_speaking = False
        else:
            self.is_speaking = True

if __name__ == "__main__":
    rospy.init_node("nav_speak")
    n = NavSpeak()
    rospy.spin()
