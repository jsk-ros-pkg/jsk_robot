#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from itertools import chain
import pymongo
from utils import UniqueStringGenerator

NameGen = UniqueStringGenerator()

class Node(object):
    def __init__(self, ntype, prop=None, name=None):
        self.parent = None
        self.children = []
        self.ntype = ntype
        self._name = name
        self.unique_name = None
        self.properties = prop or {}
    @property
    def name(self):
        if self._name is None:
            self._name = self.ntype + "_" + NameGen.gen()
        return self._name
    def add_subnode(self, n):
        self.children.append(n)
        n.parent = self
    def add_prop(self, k, v):
        self.properties.update({k:v})
    def prop_key_set(self):
        s = set(self.properties.keys())
        for c in self.children:
            s |= c.prop_key_set()
        return s
    def type_set(self):
        s = set([self.ntype])
        for c in self.children:
            s |= c.type_set()
        return s

class MongoAggregator(object):
    def __init__(self, mongo_client, task):
        self.client = mongo_client
        self.task = task
        self.root_node = None
        self.cur_node = None
    def create_root_node(self, name=None, creator="JSK", description="...", robot="PR2"):
        n = Node("RobotExperiment")
        n.add_prop("experiment", self.task)
        n.add_prop("creator", creator)
        n.add_prop("description", description)
        n.add_prop("robot", robot)
        if name is None:
            res = self.client.find_one({"_meta.task_id": self.task,
                                        "_meta.stored_type": "jsk_robot_startup/ActionEvent",
                                        "_meta.task_name": {"$exists": True }})
            if res is not None:
                n.add_prop("experimentName", res["_meta"]["task_name"])
        else:
            n.add_prop("experimentName", name)
        self.root_node = n
        return self.root_node
    def parse_action_event(self, d):
        act_name = d["name"]
        act_status = d["status"]
        act_date = d["_meta"]["inserted_at"]

        if act_status == "START":
            if act_name.startswith("pr2-logging-interface::move-to"):
                sub = Node("BaseMovement")
            elif act_name.startswith("pr2-logging-interface::angle-vector-sequence"):
                sub = Node("ArmMovement")
            elif act_name.startswith("pr2-logging-interface"):
                sub = Node(act_name)
            else:
                sub = Node("CRAMAction")
            sub.add_prop("taskContext", [act_name])
            sub.add_prop("startTime", act_date)
            sub.add_prop("goalContext", d["args"])
            if self.cur_node.children:
                prev_node = self.cur_node.children[-1]
                sub.add_prop("previousAction", prev_node.name)
                prev_node.add_prop("nextAction", sub.name)
            self.cur_node.add_subnode(sub)
            self.cur_node = sub
        else:
            self.cur_node.add_prop("actionResult", act_status)
            self.cur_node.add_prop("endTime", act_date)
            self.cur_node = self.cur_node.parent
    def parse_trajectory(self, d):
        pass
    def aggregate(self):
        self.cur_node = self.create_root_node()
        res = self.client.find({"_meta.task_id": self.task }).sort([("$_meta.inserted_at", pymongo.ASCENDING)])
        print "%d documents found" % res.count()
        for doc in res:
            t = doc["_meta"]["stored_type"]
            if t == "jsk_robot_startup/ActionEvent":
                self.parse_action_event(doc)
            elif t in ["control_msgs/FollowJointTrajectoryActionGoal",
                       "control_msgs/FollowJointTrajectoryActionResult",
                       "control_msgs/FollowJointTrajectoryActionFeedback"]:
                self.parse_trajectory(doc)
            else:
                pass
        return self.root_node
