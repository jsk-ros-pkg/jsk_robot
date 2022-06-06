#!/usr/bin/env python

import rospy
import actionlib
from sound_play.libsoundplay import SoundClient
from trigger_behavior_msgs.msg import TriggerBehaviorAction
from trigger_behavior_msgs.msg import TriggerBehaviorResult


class SpeakAndWaitBehavior(object):

    def __init__(self):

        self._speak_text = rospy.get_param('~speak_text', 'Hello World!')
        self._duraiton_wait = rospy.get_param('~duration_wait', 5.0)
        self._sound_action_name = rospy.get_param('~sound_action', 'sound_play')
        self._sound_client = SoundClient(
            blocking=True,
            sound_action=self._sound_action_name)
        self._action_server = actionlib.SimpleActionServer(
            '~behavior',
            TriggerBehaviorAction,
            self.handler,
            False)

    def handler(self, goal):

        self._sound_client.say(self._speak_text)
        rospy.sleep(self._duration_wait)

        result = TriggerBehaviorResult()
        result.success = True
        self._action_server.set_succeeded(result)


if __name__ == '__main__':

    rospy.init_node('speak_and_wait_behavior')
    node = SpeakAndWaitBehavior()
    rospy.spin()
