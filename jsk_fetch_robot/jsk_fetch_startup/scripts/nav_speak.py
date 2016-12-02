#!/usr/bin/env python

import rospy
import time

from sound_play.libsoundplay import SoundClient
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus


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

class NavSpeak:
    def __init__(self):
        self.move_base_goal_sub = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.move_base_goal_callback, queue_size = 1)
        self.move_base_result_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.move_base_result_callback, queue_size = 1)
        self.sound = SoundClient()

    def move_base_goal_callback(self, msg):
        self.sound.play(2)

    def move_base_result_callback(self, msg):
        text = "{}: {}".format(goal_status(msg.status.status), msg.status.text)
        rospy.loginfo(text)
        if msg.status.status == GoalStatus.SUCCEEDED:
            self.sound.play(1)
        elif msg.status.status == GoalStatus.PREEMPTED:
            self.sound.play(2)
        elif msg.status.status == GoalStatus.ABORTED:
            self.sound.play(3)
        else:
            self.sound.play(4)
        time.sleep(1)
        self.sound.say(text)

if __name__ == "__main__":
    global sound
    rospy.init_node("nav_speak")
    n = NavSpeak()
    rospy.spin()





