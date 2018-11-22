#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from mongodb_store.message_store import MessageStoreProxy
import mongodb_store.util as MU

class LoggerBase(object):
    def __init__(self, db_name='jsk_robot_lifelog', col_name=None, ensure_index=True):
        super(LoggerBase, self).__init__()
        self.db_name = rospy.get_param('/robot/database','jsk_robot_lifelog')
        try:
            if col_name is None:
                self.col_name = rospy.get_param('/robot/name')
            else:
                self.col_name = col_name
        except KeyError as e:
            rospy.logerr("please specify param \"/robot/name\" (e.g. pr1012, olive)")
            exit(1)

        self.task_id = None

        self.msg_store = MessageStoreProxy(database=self.db_name, collection=self.col_name)
        rospy.loginfo("connected to %s.%s" % (self.db_name, self.col_name))

        if ensure_index:
            try:
                MongoClient = MU.import_MongoClient()
                host = rospy.get_param("/mongodb_host")
                port = rospy.get_param("/mongodb_port")
                client = MongoClient(host, port)
                c = client[self.db_name][self.col_name]
                indices = [i['key'][0][0] for i in c.index_information().values()]
                keys = ["_meta.stored_type", "_meta.inserted_at"]
                for key in keys:
                    if key not in indices:
                        rospy.loginfo("Creating index for key '%s'" % key)
                        c.ensure_index(key)
                        rospy.loginfo("Created index for key '%s'" % key)
                client.close()
            except Exception as e:
                rospy.logerr("Failed to ensure index: %s" % e)

    def insert(self, msg, meta={}, wait=False):
        if self.task_id is not None:
            meta.update({ "task_id": self.task_id })
        return self.msg_store.insert(msg, meta, wait=wait)

    def spinOnce(self):
        self.task_id = rospy.get_param("/task_id", None)

