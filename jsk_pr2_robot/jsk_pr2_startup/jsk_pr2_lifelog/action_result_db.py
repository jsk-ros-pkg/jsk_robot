#!/usr/bin/python
#
# This script stores actionlib goals and results by MongoDB
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

from mongodb_store.message_store import MessageStoreProxy


class ActionResultDB(object):
    loaded_types = []
    useless_types = ['std_msgs/Header'] # message types not but action (goal|result)
    subscribers = {} # topicname:subscriber

    def __init__(self): # TODO
        self.db_name = rospy.get_param('~db_name','jsk_pr2_lifelog')
        self.col_name = rospy.get_param('~col_name', 'action_result_db')
        self.update_cycle = rospy.get_param('~update_cycle', 1.0)

        self.joint_tolerance = 1.0
        self.joint_states = []
        self.joint_states_inserted = [] # list of time stamp
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self._joint_states_cb)

        self._load_params()

        self.msg_store = MessageStoreProxy(database=self.db_name, collection=self.col_name)
        rospy.loginfo("connected to %s.%s" % (self.db_name, self.col_name))

    def _load_params(self):
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
        self._insert_action_goal(topic,type_name,msg)

    def _action_result_cb(self, topic, type_name, msg):
        self._insert_action_result(topic,type_name,msg)
        key_func = (lambda js: abs(js.header.stamp-msg.header.stamp))
        nearest_state = min(self.joint_states,key=key_func)
        self._insert_joint_states(nearest_state)

    def _joint_states_cb(self, msg):
        self.joint_states = [m for m in self.joint_states if (msg.header.stamp - m.header.stamp).to_sec() < self.joint_tolerance] + [msg]
        self.joint_states_inserted = [s for s in self.joint_states_inserted if (msg.header.stamp - s).to_sec() < self.joint_tolerance]

    # insert functions
    def _insert_action_goal(self,topic,type_name,msg):
        try:
            res = self.msg_store.insert(msg)
            rospy.loginfo("inserted action_goal message: %s (%s) -> %s", topic, type_name, res)
        except Exception as e:
            rospy.logerr("failed to insert action goal: %s (%s) -> %s", topic, type_name, e)

    def _insert_action_result(self,topic,type_name,msg):
        try:
            res = self.msg_store.insert(msg)
            rospy.loginfo("inserted action_result message: %s (%s) -> %s", topic, type_name, res)
        except Exception as e:
            rospy.logerr("failed to insert action result: %s (%s) -> %s", topic, type_name, e)
    

    def _insert_joint_states(self, msg):
        try:
            if msg.header.stamp in self.joint_states_inserted:
                return
            res = self.msg_store.insert(msg)
            rospy.loginfo("inserted joint_states message: %s", res)
        except Exception as e:
            rospy.logerr("failed to insert joint states: %s", e)

    # if the message type is goal or result, return the callback
    def _message_callback_type(self,name,type_name,type_obj):
        if not hasattr(type_obj,'header'): return None # no header message
        if type(type_obj.header) != std_msgs.msg.Header: return None # custom header message
        if hasattr(type_obj,'goal_id') and hasattr(type_obj,'goal') and type(type_obj.goal_id) == actionlib_msgs.msg.GoalID:
            return (lambda msg: self._action_goal_cb(name,type_name,msg))
        if hasattr(type_obj,'status') and hasattr(type_obj,'result') and type(type_obj.status) == actionlib_msgs.msg.GoalStatus:
            return (lambda msg: self._action_result_cb(name,type_name,msg))
        else:
            return None

    def sleep_one_cycle(self):
        rospy.sleep(self.update_cycle)

    # subscriber updater
    def update_subscribers(self):
        # check new publishers
        topics = rospy.client.get_published_topics()
        if self.action_name_white_list:
            topics = [t for t in topics if t[0] in self.action_name_white_list]
        if self.action_type_white_list:
            topics = [t for t in topics if t[1].split('/')[1] in self.action_type_white_list]
        if self.action_name_black_list:
            topics = [t for t in topics if not t[0] in self.action_name_black_list]
        if self.action_type_black_list:
            topics = [t for t in topics if not t[1].split('/')[1] in self.action_type_black_list]

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
                rospy.loginfo('imported %s.msg', pkg_name)
            except Exception as e:
                rospy.logerr('failed to import %s.msg: %s', pkg_name, e)
                rospy.logerr('please catkin_make %s', pkg_name)
                continue

            try:
                type_class = getattr(getattr(pypkg, 'msg'), msg_type)
                type_instance = type_class()
            except Exception as e:
                rospy.logerr('failed to instantiate %s.msg.%s: %s', pkg_name, msg_type, e)
                continue

            try:
                cb_obj = self._message_callback_type(topic_name, msg_type, type_instance)
                if cb_obj == None:
                    self.useless_types += [py_topic_class]
                    continue
                
                self.subscribers[topic_name] = rospy.Subscriber(topic_name, type_class, cb_obj)
                rospy.loginfo("start subscribe (topic=%s type=%s)", topic_name, msg_type)
            except Exception as e:
                self.useless_types += [py_topic_class]
                rospy.logerr('error registering subscriber: %s', e)
                continue


if __name__ == "__main__":
    rospy.init_node('action_result_db')
    obj = ActionResultDB()

    while not rospy.is_shutdown():
        obj.update_subscribers()
        obj.sleep_one_cycle()
