#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

from bson import json_util
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from mongodb_store_msgs.msg import MoveEntriesAction, MoveEntriesGoal
from mongodb_store_msgs.msg import StringList
from datetime import datetime
import pytz
from std_msgs.msg import Bool
from threading import Lock

from mongodb_store.message_store import MessageStoreProxy
from mongodb_store_msgs.msg import StringPair, StringPairList
from mongodb_store_msgs.srv import MongoQueryMsgRequest
from std_srvs.srv import Empty, EmptyResponse


class PeriodicReplicatorClient(object):
    def __init__(self):
        super(PeriodicReplicatorClient, self).__init__()

        # parameters
        self.interval = rospy.get_param("~interval", 60 * 60 * 24)  # default: 1 day
        self.delete_after_move = rospy.get_param("~delete_after_move", False)
        self.monitor_network = rospy.get_param("~monitor_network", False)
        self.poll_rate = min(300, self.interval)
        self.replicating = False
        self.lock = Lock()

        # database to be replicated
        self.database = rospy.get_param("robot/database")
        self.collections = rospy.get_param("~extra_collections", list())
        try:
            self.collections.append(rospy.get_param("robot/name"))
        except KeyError as e:
            rospy.logerr("specify param \"robot/name\" (e.g. pr1012, olive)")
            exit(1)

        # network monitoring
        self.network_connected = False
        if self.monitor_network:
            self.net_sub = rospy.Subscriber("/network/connected", Bool, self.network_connected_cb)

        # database interface
        self.date_msg_store = MessageStoreProxy(database=self.database,
                                                collection="replication")
        self.replicate_ac = actionlib.SimpleActionClient('move_mongodb_entries', MoveEntriesAction)
        for i in range(30):
            rospy.loginfo("waiting for advertise action /move_mongodb_entries ...[%d/30]" % i)
            if rospy.is_shutdown() or self.replicate_ac.wait_for_server(timeout=rospy.Duration(1)):
                break
        if not self.replicate_ac.wait_for_server(timeout=rospy.Duration(1)):
            rospy.logerr("Gave up waiting for advertise action /move_mongodb_entries")
            return

        # advertise service
        self.srv = rospy.Service("~replicate", Empty, self.replicate_srv_cb)

        self.poll_timer = rospy.Timer(rospy.Duration(self.poll_rate), self.timer_cb)

        rospy.loginfo("replication enabled: db: %s, collection: %s, interval: %d [sec]",
                      self.database, self.collections, self.interval)

    def network_ok(self):
        if self.monitor_network:
            return self.network_connected
        else:
            return True

    def get_last_replicated(self):
        try:
            last_replicated = self.date_msg_store.query(
                StringList._type, single=True, sort_query=[("$natural",-1)])
            date = last_replicated[1]["inserted_at"]
            utcdate = date.astimezone(pytz.utc).replace(tzinfo=None)
            epoch = datetime(1970, 1, 1)
            seconds = (utcdate - epoch).total_seconds()
            return rospy.Time.from_seconds(seconds)
        except Exception as e:
            rospy.logerr(e)
            return rospy.Time(0)

    def insert_replicate_date(self):
        try:
            self.date_msg_store.insert(StringList(self.collections))
        except Exception as e:
            rospy.logwarn("failed to insert last replicate date to database: %s", e)

    def move_entries(self, move_before=None, delete_after_move=False, query=None):
        move_before = move_before or rospy.Duration(self.interval)
        if query is None:
            query = {}
        query = StringPairList(
            pairs=[StringPair(first=MongoQueryMsgRequest.JSON_QUERY,
                              second=json_util.dumps(query))])
        goal = MoveEntriesGoal(database=self.database,
                               collections=StringList(self.collections),
                               move_before=move_before,
                               delete_after_move=delete_after_move,
                               query=query)
        self.replicate_ac.send_goal(goal,
                                    done_cb=self.done_cb,
                                    active_cb=self.active_cb,
                                    feedback_cb=self.feedback_cb)
        while not self.replicate_ac.wait_for_result(timeout=rospy.Duration(5.0)):
            if rospy.is_shutdown():
                break
            elif self.monitor_network and not self.network_connected:
                rospy.loginfo("disconnected wired network connection. canceling replication...")
                self.replicate_ac.cancel_all_goals()

    def do_replicate(self):
        with self.lock:
            try:
                self.replicating = True
                rospy.loginfo("Starting to replicate persistent data")
                # first replicate persistent data without removal
                move_before = rospy.Duration(self.interval)
                self.move_entries(move_before=move_before,
                                  delete_after_move=False,
                                  query={"_meta.persistent": True})
                # then replicate all data with removal
                rospy.loginfo("Starting to replicate target data")
                self.move_entries(move_before=move_before,
                                  delete_after_move=self.delete_after_move,
                                  query={"_meta.persistent": {"$exists": False}})
            except Exception as e:
                rospy.logerr("Replication failed: %s" % e)
            finally:
                self.replicating = False

    def timer_cb(self, event=None):
        if self.replicate_ac.get_state() == GoalStatus.ACTIVE:
            return
        if not self.network_ok():
            rospy.loginfo("No network connection. Skipping replication.")
            return

        try:
            now = event.current_real
        except:
            now = rospy.Time.now()

        last_replicated = self.get_last_replicated()
        if (now - last_replicated).to_sec() < self.interval:
            rospy.loginfo("Interval %d < %d. skipping replication.",
                          (now - last_replicated).to_sec(), self.interval)
            return
        if not self.replicating:
            self.do_replicate()

    def replicate_srv_cb(self, req):
        if self.replicating:
            rospy.loginfo("Replication is already started")
        else:
            self.do_replicate()
        return EmptyResponse()

    def done_cb(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Replication suceeded")
            self.insert_replicate_date()
        else:
            rospy.logwarn("Replication finished with status %s" % actionlib.get_name_of_constant(GoalStatus, status))

    def active_cb(self):
        if not self.network_ok():
            rospy.loginfo("disconnected wired network connection. canceling replication...")
            self.replicate_ac.cancel_all_goals()

    def feedback_cb(self, feedback):
        rospy.loginfo(feedback)

    def network_connected_cb(self, msg):
        self.network_connected = msg.data


if __name__ == '__main__':
    rospy.init_node("mongodb_replicator_client")
    r = PeriodicReplicatorClient()
    rospy.spin()
