#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from importlib import import_module

import rospy
import actionlib
from sound_play.msg import SoundRequest
from sound_play.msg import SoundRequestGoal
from sound_play.msg import SoundRequestAction
from std_msgs.msg import Empty


def expr_eval(expr):
    def eval_fn(topic, m, t):
        return eval(expr)
    return eval_fn


class Shutdown(object):
    """
    This node shuts down or reboots the robot itself
    according to the rostopic.

    Note that this node needs to be run with sudo privileges.

    Usage:
    # Launch node
    $ su [sudo user] -c ". [setup.bash]; rosrun jsk_robot_startup shutdown.py"

    # To shutdown robot
    rostopic pub /shutdown std_msgs/Empty
    # To restart robot
    rostopic pub /reboot std_msgs/Empty
    """

    def __init__(self):
        rospy.loginfo('Start shutdown node.')
        self.client = actionlib.SimpleActionClient(
            '/robotsound_jp', SoundRequestAction)
        self.condition = rospy.get_param(
            '~input_condition', None)
        if self.condition is not None:
            self.topic_name = rospy.resolve_name('~input')
            self.filter_fn = expr_eval(self.condition)
            self.last_received_topic = None
            self.sub_check_topic = rospy.Subscriber(
                self.topic_name,
                rospy.AnyMsg,
                callback=self.callback,
                queue_size=1)

        rospy.Subscriber('shutdown', Empty, self.shutdown)
        rospy.Subscriber('reboot', Empty, self.reboot)
        self.shutdown_command = rospy.get_param(
            '~shutdown_command', '/sbin/shutdown -h now')
        self.reboot_command = rospy.get_param(
            '~reboot_command', '/sbin/shutdown -r now')
        self.shutdown_sentence = rospy.get_param(
            '~shutdown_sentence', 'シャットダウンします。')
        self.reboot_sentence = rospy.get_param(
            '~reboot_sentence', '再起動します。')
        self.lang = rospy.get_param('~lang', 'jp')
        self.volume = rospy.get_param('~volume', 1.0)
        self.timeout_server = rospy.get_param('~timeout_server', 1.0)
        self.timeout_result = rospy.get_param('~timeout_result', 10.0)

    def speak(self, text):
        timeout_server = rospy.Duration(self.timeout_server)
        timeout_result = rospy.Duration(self.timeout_result)
        if self.client.wait_for_server(timeout=timeout_server):
            msg = SoundRequest()
            msg.sound = SoundRequest.SAY
            msg.command = SoundRequest.PLAY_ONCE
            msg.volume = max(0, min(1, self.volume))
            msg.arg = text
            msg.arg2 = self.lang
            goal = SoundRequestGoal(sound_request=msg)
            self.client.send_goal(goal)
            self.client.wait_for_result(timeout=timeout_result)
            return self.client.get_result()

    def callback(self, msg):
        if isinstance(msg, rospy.msg.AnyMsg):
            package, msg_type = msg._connection_header['type'].split('/')
            ros_pkg = package + '.msg'
            msg_class = getattr(import_module(ros_pkg), msg_type)
            self.sub_check_topic.unregister()
            deserialized_sub = rospy.Subscriber(
                self.topic_name, msg_class, self.callback)
            self.sub_check_topic = deserialized_sub
            msg = msg_class().deserialize(msg._buff)
        self.last_received_topic = self.topic_name, msg, rospy.Time.now()

    def shutdown(self, msg):
        if self.condition is not None:
            if self.last_received_topic is None:
                rospy.loginfo(
                    'received shutdown. '
                    'However input topic "{}" has not been received.'
                    .format(self.topic_name))
                return
            topic_name, m, t = self.last_received_topic
            if self.filter_fn(topic_name, m, t) is False:
                rospy.loginfo(
                    'received shutdown. '
                    'However condition "{}" is not satisfied.'
                    .format(self.condition))
                return

        rospy.loginfo('Shut down robot.')
        self.speak(self.shutdown_sentence)
        ret = os.system(self.shutdown_command)
        if ret != 0:
            rospy.logerr("Failed to call '$ {}'. Check authentication.".format(
                self.shutdown_command))

    def reboot(self, msg):
        rospy.loginfo('Reboot robot.')
        self.speak(self.reboot_sentence)
        ret = os.system(self.reboot_command)
        if ret != 0:
            rospy.logerr("Failed to call '$ {}'. Check authentication.".format(
                self.reboot_command))


if __name__ == '__main__':
    rospy.init_node('shutdown')
    s = Shutdown()
    rospy.spin()
