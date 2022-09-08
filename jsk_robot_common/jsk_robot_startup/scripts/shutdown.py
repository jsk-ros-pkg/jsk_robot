#!/usr/bin/env python

import os
from importlib import import_module

import rospy
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
        ret = os.system(self.shutdown_command)
        if ret != 0:
            rospy.logerr("Failed to call '$ {}'. Check authentication.".format(
                self.shutdown_command))

    def reboot(self, msg):
        rospy.loginfo('Reboot robot.')
        ret = os.system(self.reboot_command)
        if ret != 0:
            rospy.logerr("Failed to call '$ {}'. Check authentication.".format(
                self.reboot_command))


if __name__ == '__main__':
    rospy.init_node('shutdown')
    s = Shutdown()
    rospy.spin()
