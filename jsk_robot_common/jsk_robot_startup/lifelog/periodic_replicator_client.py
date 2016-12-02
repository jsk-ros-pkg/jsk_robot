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
from std_msgs.msg import Bool

from mongodb_store.message_store import MessageStoreProxy

class PeriodicReplicatorClient(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.dead = Event()
        self.replicate_interval = rospy.get_param("replication/interval", 60 * 60 * 24) # default: 1 day
        self.delete_after_move = rospy.get_param("replication/delete_after_move", False)
        self.database = rospy.get_param("robot/database")
        self.collections = rospy.myargv()[1:]
        try:
            self.collections.append(rospy.get_param("robot/name"))
        except KeyError as e:
            rospy.logerr("specify param \"robot/name\" (e.g. pr1012, olive)")
            exit(1)

        self.periodic = rospy.get_param("replication/periodic", True)
        if self.periodic:
            rospy.loginfo("periodic replication interval: %d [sec]", self.replicate_interval)
            self.disable_on_wireless_network = rospy.get_param("replication/disable_on_wireless_network", False)
            if self.disable_on_wireless_network:
                self.network_connected = False
                self.net_sub = rospy.Subscriber("/network/connected", Bool, self.network_connected_cb)
        else:
            self.replicate_interval = 1

        self.date_msg_store = MessageStoreProxy(database=self.database,
                                                collection="replication")
        self.replicate_ac = actionlib.SimpleActionClient('move_mongodb_entries', MoveEntriesAction)
        rospy.loginfo("waiting for service advertise /move_mongodb_entries ...")
        self.replicate_ac.wait_for_server()
        rospy.loginfo("replication enabled: db: %s, collection: %s, periodic: %s",
                      self.database, self.collections, self.periodic)

    def run(self):
        while not self.dead.wait(self.replicate_interval):
            if self.disable_on_wireless_network and not self.network_connected:
                rospy.loginfo("no wired network connection. skipping replication...")
            else:
                move_before = self.time_after_last_replicate_date()
                self.move_entries(move_before)
                self.insert_replicate_date()
                if not self.periodic:
                    self.exit()

    def time_after_last_replicate_date(self):
        delta = 0
        try:
            last_replicated = self.date_msg_store.query(StringList._type, single=True, sort_query=[("$natural",-1)])
            date = last_replicated[1]["inserted_at"]
            rospy.loginfo("last replicated at %s", date)
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
        goal = MoveEntriesGoal(database=self.database,
                               collections=StringList(self.collections),
                               move_before=move_before,
                               delete_after_move=self.delete_after_move)
        self.replicate_ac.send_goal(goal,
                                    active_cb=self.active_cb,
                                    feedback_cb=self.feedback_cb)
        while not self.replicate_ac.wait_for_result(timeout=rospy.Duration(5.0)):
            if self.disable_on_wireless_network and not self.network_connected:
                rospy.loginfo("disconnected wired network connection. canceling replication...")
                self.cancel()


    def active_cb(self):
        if self.disable_on_wireless_network and not self.network_connected:
                rospy.loginfo("disconnected wired network connection. canceling replication...")
                self.cancel()

    def feedback_cb(self, feedback):
        rospy.loginfo(feedback)

    def cancel(self):
        self.replicate_ac.cancel_all_goals()

    def exit(self):
        self.cancel()
        self.dead.set()

    def network_connected_cb(self, msg):
        self.network_connected = msg.data

if __name__ == '__main__':
    rospy.init_node("mongodb_replicator_client")
    r = PeriodicReplicatorClient()
    rospy.on_shutdown(r.cancel)
    r.start()
