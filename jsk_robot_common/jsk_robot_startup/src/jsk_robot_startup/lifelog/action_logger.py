#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
#
# This script stores actionlib goals, results and feedbacks by MongoDB
#

from collections import defaultdict
import re
import rospy

import std_msgs.msg
import actionlib_msgs.msg
from actionlib_msgs.msg import GoalID, GoalStatus

from .logger_base import LoggerBase


class ActionLogger(LoggerBase):
    loaded_types = []
    useless_types = ['std_msgs/Header'] # message types not but action (goal|result)
    subscribers = {} # topicname:subscriber

    def __init__(self):
        LoggerBase.__init__(self)

        self.queue_size = rospy.get_param("~queue_size", 30)
        self.action_regex = re.compile(".*Action(Result|Goal|Feedback)$")
        self.update_rate = rospy.get_param("~update_rate", 1.0)
        self.max_rate = rospy.get_param("~max_rate", 3.0)
        self.last_inserted_time = defaultdict(rospy.Time)

        self.load_params()

    def load_params(self):
        try:
            self.action_name_white_list = rospy.get_param('~white_list')['name']
            rospy.loginfo("whitelist_name: %s", self.action_name_white_list)
        except:
            self.action_name_white_list = None
        try:
            self.action_name_black_list = rospy.get_param('~black_list')['name']
            rospy.loginfo("blacklist_name: %s", self.action_name_black_list)
        except:
            self.action_name_black_list = None
        try:
            self.action_type_white_list = rospy.get_param('~white_list')['type']
            rospy.loginfo("whitelist_type: %s", self.action_type_white_list)
        except:
            self.action_type_white_list = None
        try:
            self.action_type_black_list = rospy.get_param('~black_list')['type']
            rospy.loginfo("blacklist_name: %s", self.action_type_black_list)
        except:
            self.action_type_black_list = None

    # callback functions
    def _action_goal_cb(self, topic, type_name, msg):
        self._insert(topic, type_name, msg, True)

    def _action_result_cb(self, topic, type_name, msg):
        self._insert(topic, type_name, msg, True)

    def _action_feedback_cb(self, topic, type_name, msg):
        self._insert(topic, type_name, msg, False)

    # insert functions
    def _insert(self, topic, type_name, msg, force):
        key = topic + '-' + type_name
        stamp = msg.header.stamp
        last = self.last_inserted_time[key]
        dt = (stamp - last).to_sec()

        if not force and dt < 1.0 / self.max_rate:
            return True  # drop

        try:
            res = self.insert(msg, meta={'input_topic': topic})
            self.last_inserted_time[key] = stamp
            rospy.logdebug("inserted message: %s (%s) -> %s", topic, type_name, res)
            return True
        except Exception as e:
            rospy.logerr("failed to insert message: %s (%s) -> %s", topic, type_name, e)
            return False

    # if the message type is goal or result, return the callback
    def _get_callback(self, name, type_name, type_obj):
        if not hasattr(type_obj, 'header'):
            # ignore message without header
            return None
        if type(type_obj.header) != std_msgs.msg.Header:
            # ignore message without non-standard header
            return None
        if hasattr(type_obj, 'goal_id') and hasattr(type_obj, 'goal') and isinstance(type_obj.goal_id, GoalID):
            # action goal
            return (lambda msg: self._action_goal_cb(name, type_name, msg))
        elif hasattr(type_obj, 'status') and isinstance(type_obj.status, GoalStatus):
            if hasattr(type_obj, 'result'):
                # action result
                return (lambda msg: self._action_result_cb(name, type_name, msg))
            elif hasattr(type_obj, 'feedback'):
                # action feedback
                return (lambda msg: self._action_feedback_cb(name, type_name, msg))
        return None

    # subscriber updater
    def update_subscribers(self):
        # check new publishers
        topics = [t for t in rospy.get_published_topics() if self.action_regex.match(t[1])]
        if self.action_name_white_list:
            topics = [t for t in topics if t[0] in self.action_name_white_list]
        if self.action_type_white_list:
            topics = [t for t in topics if t[1] in self.action_type_white_list]
        if self.action_name_black_list:
            topics = [t for t in topics if not t[0] in self.action_name_black_list]
        if self.action_type_black_list:
            topics = [t for t in topics if not t[1] in self.action_type_black_list]

        for topic_name, topic_type in topics:
            (pkg_name, msg_type)  = topic_type.split('/')
            py_topic_class = '%s.msg.%s' % (pkg_name, msg_type)
            if py_topic_class in self.useless_types:
                continue
            if topic_name in self.subscribers.keys():
                continue

            # Import and Check
            try:
                pypkg = __import__('%s.msg' % pkg_name)
                rospy.logdebug('imported %s.msg', pkg_name)
            except Exception as e:
                rospy.logerr('failed to import %s.msg: %s', pkg_name, e)
                rospy.logerr('please catkin_make %s', pkg_name)
                self.useless_types += [py_topic_class]
                continue

            try:
                type_class = getattr(getattr(pypkg, 'msg'), msg_type)
                type_instance = type_class()
            except Exception as e:
                rospy.logerr('failed to instantiate %s.msg.%s: %s', pkg_name, msg_type, e)
                self.useless_types += [py_topic_class]
                continue

            try:
                cb_func = self._get_callback(topic_name, msg_type, type_instance)
                if cb_func is None:
                    self.useless_types += [py_topic_class]
                    continue
                self.subscribers[topic_name] = rospy.Subscriber(topic_name, type_class,
                                                                cb_func, queue_size=self.queue_size)
                rospy.logdebug("start subscribe (topic=%s type=%s)", topic_name, msg_type)
            except Exception as e:
                self.useless_types += [py_topic_class]
                rospy.logerr('error registering subscriber: %s', e)
                continue

        # cleanup subscribers
        cleanup_topics = set(self.subscribers.keys()) - set([t[0] for t in topics])
        for t in cleanup_topics:
            self.subscribers[t].unregister()
            del self.subscribers[t]
            rospy.logdebug("unsubscribe %s" % t)

    def run(self):
        r = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.update_subscribers()
            self.spinOnce()
            r.sleep()

if __name__ == "__main__":
    rospy.init_node('action_logger')
    ActionLogger().run()
