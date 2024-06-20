#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import copy
import math
import datetime
import genpy
import numpy as np
import rospy
from robot_state_publisher.srv import (GetRobotTransforms, GetRobotTransformsRequest)
from tf import transformations as T


def datetime_to_rostime_json(dt):
    total = (dt - datetime.datetime.utcfromtimestamp(0)).total_seconds()
    secs = int(total)
    nsecs = int((total - secs) * 1000000000)
    return { u"secs": secs, u"nsecs": nsecs }

def datetime_to_date_json(dt):
    total = (dt - datetime.datetime.utcfromtimestamp(0)).total_seconds()
    return { "$date": int(math.floor(total * 1000)) }

def rostime_to_date_json(t):
    return { "$date": int(math.floor(t.to_sec() * 1000)) }

def rostime_to_rostime_json(t):
    return { u"secs": t.secs, u"nsecs": t.nsecs }

def rostime_json_to_date_json(j):
    return rostime_to_date_json(rospy.Time(j["secs"], j["nsecs"]))

def date_json_to_rostime_json(d):
    sec = d["$date"] * 0.001
    secs = int(sec)
    nsecs = int((sec - secs) * 1000000000)
    return { u"secs": secs, u"nsecs": nsecs }

def convert_stamp(o, use_rostime):
    if use_rostime:
        if isinstance(o, datetime.datetime):
            return datetime_to_rostime_json(o)
        elif isinstance(o, rospy.Time) or isinstance(o, genpy.Time):
            return rostime_to_rostime_json(o)
        elif isinstance(o, dict) and "$date" in o:
            return date_json_to_rostime_json(o)
        else:
            return o
    else:
        if isinstance(o, datetime.datetime):
            return datetime_to_date_json(o)
        elif isinstance(o, rospy.Time) or isinstance(o, genpy.Time):
            return rostime_to_date_json(o)
        elif isinstance(o, dict) and "secs" in o:
            return rostime_json_to_date_json(o)
        else:
            return o

class BSONConversion(object):
    def __init__(self, srv_name="get_transforms"):
        self.get_transforms_srv = rospy.ServiceProxy(srv_name, GetRobotTransforms)
        self.cached_transforms = []

    def offset_map_frame(self, doc, offset):
        if len(offset) == 6:
            trans_rot_mat = T.euler_matrix(*offset[3:])
        elif len(offset) == 7:
            trans_rot_mat = T.quaternion_matrix(offset[3:])
        else:
            raise ValueError("orientation must be rpy or quat")
        trans_mat = np.dot(T.translation_matrix(tuple(offset[:3])),
                           trans_rot_mat)
        t = T.translation_from_matrix(trans_mat)
        q = T.quaternion_from_matrix(trans_mat)
        # /map -> /room
        room = {}
        room["header"] = copy.deepcopy(doc["transforms"][0]["header"])
        room["header"]["frame_id"] = "map"
        room["child_frame_id"] = "room"
        room["transform"] = {
            "translation": { "x": t[0], "y": t[1], "z": t[2] },
            "rotation": { "x": q[0], "y": q[1], "z": q[2], "w": q[3] } }

        for i, d in enumerate(doc["transforms"]):
            if d["header"]["frame_id"] != "map":
                continue
            else:
                doc["transforms"][i]["header"]["frame_id"] = "room"
        doc["transforms"].append(room)
        return doc

    def append_cached_transforms(self, data):
        if len(data["transforms"]) == 0:
            return data
        stamp = copy.deepcopy(data["transforms"][0]["header"]["stamp"])
        tf_added = []
        for c in self.cached_transforms:
            append = True
            for d in data["transforms"]:
                if d["header"]["frame_id"] == c["header"]["frame_id"] and d["child_frame_id"] == c["child_frame_id"]:
                    append = False
                    break
            if append:
                c["header"]["stamp"] = stamp
                tf_added.append(c)
        data["transforms"] += tf_added
        self.cached_transforms = copy.deepcopy(data["transforms"])
        return data

    def to_tf_json(self, doc, use_rostime=False, offset=[0,0,0,0,0,0,0], export_meta=False):
        t = doc["_meta"]["stored_type"]
        if   t == "posedetection_msgs/Object6DPose":
            ret = self.object6dpose_to_tf(doc, use_rostime)
        elif t == "geometry_msgs/TransformStamped":
            ret = self.transform_stamped_to_tf(doc, use_rostime)
        elif t == "move_base_msgs/MoveBaseActionFeedback":
            ret = self.move_base_action_feedback_to_tf(doc, use_rostime)
        elif t == "control_msgs/FollowJointTrajectoryActionFeedback":
            ret = self.follow_joint_trajectory_feedback_to_tf(doc, use_rostime)
        elif t == "sensor_msgs/JointState":
            ret = self.joint_state_to_tf(doc, use_rostime)
        elif t == "tf2_msgs/TFMessage":
            ret = self.tf_message_to_tf(doc, use_rostime)

        ret["__recorded"] = datetime_to_date_json(ret["_meta"]["inserted_at"])
        ret["__topic"] = "/tf"
        if not export_meta:
            del ret["_meta"]

        data = self.offset_map_frame(ret, offset)
        data = self.append_cached_transforms(data)

        return data

    def tf_message_to_tf(self, msg, use_rostime):
        for i, t in enumerate(msg["transforms"]):
            msg["transforms"][i]["header"]["stamp"] = convert_stamp(t["header"]["stamp"], use_rostime)
        return msg

    def transform_stamped_array_to_tf(self, ts_arr, use_rostime):
        meta = ts_arr[0].pop("_meta")
        for ts in ts_arr:
            if "_meta" in ts:
                del ts["_meta"]
            if "_id" in ts:
                del ts["_id"]
        ret = { "_meta": meta,
                "transforms": ts_arr }
        for i in range(len(ret["transforms"])):
            ret["transforms"][i]["header"]["stamp"] = convert_stamp(ret["transforms"][i]["header"]["stamp"], use_rostime)
            ret["transforms"][i]["header"]["frame_id"] = ret["transforms"][i]["header"]["frame_id"]
            ret["transforms"][i]["child_frame_id"] = ret["transforms"][i]["child_frame_id"]
        return ret

    def transform_stamped_to_tf(self, ts, use_rostime):
        return self.transform_stamped_array_to_tf([ts], use_rostime)

    def object6dpose_to_tf(self, od, use_rostime, robot_frame_id="base_footprint"):
        return {
            "_meta": od["_meta"],
            "transforms": [{
                "header": {
                    "frame_id": robot_frame_id,
                    "stamp": convert_stamp(od["_meta"]["inserted_at"], use_rostime),
                    "seq": 0,
                },
                "child_frame_id": od["type"],
                "transform": {
                    "translation": od["pose"]["position"],
                    "rotation": od["pose"]["orientation"],
                },
            }]}

    def move_base_action_feedback_to_tf(self, f, use_rostime, robot_frame_id="/base_footprint"):
        base = f["feedback"]["base_position"]
        base["header"]["frame_id"] = base["header"]["frame_id"]
        base["header"]["stamp"] = convert_stamp(base["header"]["stamp"], use_rostime)
        return {
            "_meta": f["_meta"],
            "transforms": [{
                "header": base["header"],
                "child_frame_id": robot_frame_id,
                "transform": {
                    "translation": base["pose"]["position"],
                    "rotation": base["pose"]["orientation"],
                },
            }]}

    def joint_state_to_tf(self, js, use_rostime):
        req = GetRobotTransformsRequest()
        req.joint_states.header.stamp.secs = js["header"]["stamp"]["secs"]
        req.joint_states.header.stamp.nsecs = js["header"]["stamp"]["nsecs"]
        req.joint_states.header.seq = js["header"]["seq"]
        req.joint_states.name = js["name"]
        req.joint_states.position = js["position"]
        req.joint_states.velocity = js["velocity"]
        req.joint_states.effort = js["effort"]
        res = self.get_transforms_srv(req).transforms
        return {
            "_meta": js["_meta"],
            "transforms": [{
                "header": {
                    "seq": t.header.seq,
                    "frame_id": t.header.frame_id,
                    "stamp": convert_stamp(t.header.stamp, use_rostime),
                },
                "child_frame_id": t.child_frame_id,
                "transform": {
                    "translation": {
                        "x": t.transform.translation.x,
                        "y": t.transform.translation.y,
                        "z": t.transform.translation.z },
                    "rotation": {
                        "x": t.transform.rotation.x,
                        "y": t.transform.rotation.y,
                        "z": t.transform.rotation.z,
                        "w": t.transform.rotation.w }}} for t in res]}

    def follow_joint_trajectory_feedback_to_tf(self, f, use_rostime):
        fb = f["feedback"]
        req = GetRobotTransformsRequest()
        req.joint_states.header.stamp.secs = fb["header"]["stamp"]["secs"]
        req.joint_states.header.stamp.nsecs = fb["header"]["stamp"]["nsecs"]
        req.joint_states.header.seq = fb["header"]["seq"]
        req.joint_states.name = fb["joint_names"]
        req.joint_states.position = fb["actual"]["positions"]
        req.joint_states.velocity = fb["actual"]["velocities"]
        res = self.get_transforms_srv(req).transforms
        return {
            "_meta": f["_meta"],
            "transforms": [{
                "header": {
                    "seq": t.header.seq,
                    "frame_id": t.header.frame_id,
                    "stamp": convert_stamp(t.header.stamp, use_rostime),
                },
                "child_frame_id": t.child_frame_id,
                "transform": {
                    "translation": {
                        "x": t.transform.translation.x,
                        "y": t.transform.translation.y,
                        "z": t.transform.translation.z },
                    "rotation": {
                        "x": t.transform.rotation.x,
                        "y": t.transform.rotation.y,
                        "z": t.transform.rotation.z,
                        "w": t.transform.rotation.w }}} for t in res]}
