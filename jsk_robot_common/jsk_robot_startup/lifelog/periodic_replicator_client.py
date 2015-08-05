#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

import rospy
import actionlib
from mongodb_store_msgs.msg import  MoveEntriesAction, MoveEntriesGoal, StringList
import datetime as dt
from threading import Thread, Event
import sys
import signal
import time

from mongodb_store.message_store import MessageStoreProxy

class PeriodicReplicatorClient(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.dead = Event()
        self.interval = rospy.get_param("mongodb_replication_interval", 60 * 60 * 24) # default: 1 day
        self.delete_after_move = rospy.get_param("mongodb_replication_delete_after_move", False)
        self.replicate_interval = rospy.Duration(self.interval)
        self.database = rospy.get_param("robot/database")
        self.collections = sys.argv[2:]
        try:
            self.collections.append(rospy.get_param("robot/name"))
        except KeyError as e:
            rospy.logerr("specify param \"robot/name\" (e.g. pr1012, olive)")
            exit(1)
        self.periodic = rospy.get_param("~periodic", True)
        self.date_msg_store = MessageStoreProxy(database=self.database,
                                                collection="replication")

    def run(self):
        while not self.dead.wait(self.interval):
            move_before = self.time_after_last_replicate_date()
            self.move_entries(move_before)
            self.insert_replicate_date()

    def time_after_last_replicate_date(self):
        delta = 60 * 60 * 24 # 1 day
        try:
            last_replicated = self.date_msg_store.query(StringList._type, single=True, sort_query=[("$natural",-1)])
            date = last_replicated[1]["inserted_at"]
            rospy.loginfo("last replicated at %s", date)
            delta = (dt.datetime.now() - date).seconds + 60
        except Exception as e:
            rospy.logwarn("failed to search last replicated date from database: %s", e)
        finally:
            return rospy.Duration(delta)

    def insert_replicate_date(self):
        try:
            self.date_msg_store.insert(StringList(self.collections))
        except Exception as e:
            rospy.logwarn("failed to insert last replicate date to database: %s", e)

    def move_entries(self, move_before):
        client = actionlib.SimpleActionClient('move_mongodb_entries', MoveEntriesAction)
        client.wait_for_server()
        goal = MoveEntriesGoal(database=self.database,
                               collections=StringList(self.collections),
                               move_before=move_before,
                               delete_after_move=self.delete_after_move)
        client.send_goal(goal, feedback_cb=self.feedback_cb)
        client.wait_for_result()

    def feedback_cb(self, feedback):
        rospy.loginfo(feedback)

    def cancel(self):
        self.dead.set()

if __name__ == '__main__':
    rospy.init_node("mongodb_replicator_client")
    r = PeriodicReplicatorClient()
    rospy.on_shutdown(r.cancel)
    r.start()
