#!/usr/bin/python
#
# This script stores actionlib goals, results and feedbacks by MongoDB
# And store the current robot joints when recieve result.
#
# table configuration is bottom of this script
#

# parameters:
#    joint_tolerance: store the past joint_states (sec)

import rospy
import tf

import std_msgs.msg
import actionlib_msgs.msg
import sensor_msgs.msg
from sensor_msgs.msg import JointState

from logger_base import LoggerBase


class ActionLogger(LoggerBase):
    loaded_types = []
    useless_types = ['std_msgs/Header'] # message types not but action (goal|result)
    subscribers = {} # topicname:subscriber

    def __init__(self):
        LoggerBase.__init__(self)
        self.joint_tolerance = 1.0
        self.joint_states = []
        self.joint_states_inserted = [] # list of time stamp
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.__joint_states_cb)

        self.__load_params()

    def __load_params(self):
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
    def __action_goal_cb(self, topic, type_name, msg):
        self.__insert_action_goal(topic, type_name, msg)

    def __action_result_cb(self, topic, type_name, msg):
        self.__insert_action_result(topic, type_name, msg)
        key_func = (lambda js: abs(js.header.stamp-msg.header.stamp))
        nearest_state = min(self.joint_states, key=key_func)
        self.__insert_joint_states(nearest_state)

    def __action_feedback_cb(self, topic, type_name, msg):
        self.__insert_action_feedback(topic, type_name, msg)

    def __joint_states_cb(self, msg):
        self.joint_states = [m for m in self.joint_states if (msg.header.stamp - m.header.stamp).to_sec() < self.joint_tolerance] + [msg]
        self.joint_states_inserted = [s for s in self.joint_states_inserted if (msg.header.stamp - s).to_sec() < self.joint_tolerance]

    # insert functions
    def __insert_action_goal(self, topic, type_name, msg):
        try:
            res = self.insert(msg)
            rospy.logdebug("inserted action_goal message: %s (%s) -> %s", topic, type_name, res)
        except Exception as e:
            rospy.logerr("failed to insert action goal: %s (%s) -> %s", topic, type_name, e)

    def __insert_action_result(self, topic, type_name, msg):
        try:
            res = self.insert(msg)
            rospy.logdebug("inserted action_result message: %s (%s) -> %s", topic, type_name, res)
        except Exception as e:
            rospy.logerr("failed to insert action result: %s (%s) -> %s", topic, type_name, e)

    def __insert_action_feedback(self, topic, type_name, msg):
        try:
            res = self.insert(msg)
            rospy.logdebug("inserted action_feedback message: %s (%s) -> %s", topic, type_name, res)
        except Exception as e:
            rospy.logerr("failed to insert action goal: %s (%s) -> %s", topic, type_name, e)

    def __insert_joint_states(self, msg):
        try:
            if msg.header.stamp in self.joint_states_inserted:
                return
            res = self.insert(msg)
            rospy.logdebug("inserted joint_states message: %s", res)
        except Exception as e:
            rospy.logerr("failed to insert joint states: %s", e)

    # if the message type is goal or result, return the callback
    def __message_callback_type(self, name, type_name, type_obj):
        if not hasattr(type_obj,'header'): return None # ignore no header message
        if type(type_obj.header) != std_msgs.msg.Header: return None # ignore non-standard header message
        if hasattr(type_obj,'goal_id') and hasattr(type_obj,'goal') and type(type_obj.goal_id) == actionlib_msgs.msg.GoalID:
            return (lambda msg: self.__action_goal_cb(name,type_name,msg)) # action goal
        elif hasattr(type_obj,'status') and isinstance(type_obj.status, actionlib_msgs.msg.GoalStatus):
            if hasattr(type_obj,'result'):
                return (lambda msg: self.__action_result_cb(name,type_name,msg)) # action result
            elif hasattr(type_obj, 'feedback'):
                return (lambda msg: self.__action_feedback_cb(name, type_name, msg)) # action feedback
        return None

    # subscriber updater
    def update_subscribers(self):
        # check new publishers
        topics = rospy.client.get_published_topics()
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
                cb_func = self.__message_callback_type(topic_name, msg_type, type_instance)
                if cb_func is None:
                    self.useless_types += [py_topic_class]
                    continue
                self.subscribers[topic_name] = rospy.Subscriber(topic_name, type_class, cb_func, queue_size=30)
                rospy.loginfo("start subscribe (topic=%s type=%s)", topic_name, msg_type)
            except Exception as e:
                self.useless_types += [py_topic_class]
                rospy.logerr('error registering subscriber: %s', e)
                continue

    def run(self):
        while not rospy.is_shutdown():
            self.update_subscribers()
            self.spinOnce()


if __name__ == "__main__":
    rospy.init_node('action_logger')
    ActionLogger().run()
