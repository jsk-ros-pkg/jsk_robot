#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
from mongodb_store.message_store import MessageStoreProxy


class LoggerBase(object):
    def __init__(self, db_name='jsk_robot_lifelog', col_name=None):
        self.db_name = rospy.get_param('robot/database','jsk_robot_lifelog')
        try:
            if col_name is None:
                self.col_name = rospy.get_param('robot/name')
            else:
                self.col_name = col_name
        except KeyError as e:
            rospy.logerr("please specify param \"robot/name\" (e.g. pr1012, olive)")
            exit(1)
        self.update_cycle = rospy.get_param("update_cycle", 1)

        self.task_id = None

        self.msg_store = MessageStoreProxy(database=self.db_name, collection=self.col_name)
        rospy.loginfo("connected to %s.%s" % (self.db_name, self.col_name))

    def insert(self, msg, meta={}):
        if self.task_id is not None:
            meta.update({ "TASK_ID": self.task_id })
        return self.msg_store.insert(msg, meta)

    def spinOnce(self):
        rospy.sleep(self.update_cycle)
        self.task_id = rospy.get_param("/task_id", None)

