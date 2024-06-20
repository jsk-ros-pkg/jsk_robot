#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import argparse
import bson
import datetime
import json
import os
import rospy
import sys
import time
from jsk_owl_exporter import *


def json_serializer(o):
    if isinstance(o, bson.ObjectId):
        return { "$oid": str(o) }
    elif isinstance(o, datetime.datetime):
        return { "$date": datetime_to_epoch_time(o) }
    raise TypeError(repr(o) + " is not JSON serializable")

def export_tf(db_addr, task_id, out_dir, srv_name, offset, debug):
    client = get_mongo_client(db_addr)
    bson_cvt = BSONConversion(srv_name)
    cur = client.find({
        "_meta.stored_type": { "$in": [
            "geometry_msgs/TransformStamped",
            "posedetection_msgs/Object6DPose",
            "move_base_msgs/MoveBaseActionFeedback",
            "control_msgs/FollowJointTrajectoryActionFeedback",
            "sensor_msgs/JointState",
            "tf2_msgs/TFMessage",
        ]},
        "_meta.task_id": task_id,
    }).sort("_id")

    if not os.path.exists(out_dir):
        print "%s does not exists. created directories..." % out_dir
        os.makedirs(out_dir)

    out_path = os.path.join(out_dir, "tf.json")
    msg_count = cur.count()
    print "exporting %s messages to %s" % (msg_count, out_path)

    prev_time = time.time()
    with open(out_path, "w") as f:
        for i, d in enumerate(cur):
            if time.time() - prev_time > 2:
                print "%f%% done." % (i*100.0/msg_count)
                prev_time = time.time()
            t = d["_meta"]["stored_type"]
            j = bson_cvt.to_tf_json(d, offset=offset, export_meta=debug)
            json.dump(j, f, default=json_serializer)
            f.write(os.linesep)
    print "All messages were exported."
    return True

def exec_command(args):
    rospy.init_node("export_transform")
    return export_tf(args.db, args.task, args.output, args.srv, args.offset, args.debug)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description="OWL Transform Information Exporter from mongo database")
    p.add_argument("task", type=str, help="Task ID")
    p.add_argument("-o", "--output", type=str, help="output dir",
                   default=os.getcwd())
    p.add_argument("-d", "--debug", action='store_true',
                   default=False, help="export with debug information")

    # mongodb
    p.add_argument("--db", default="mongodb://localhost:27017/jsk_robot_lifelog/pr1012",
                   type=str, help="address for database")

    # robot_state_publisher
    p.add_argument("--srv", default="get_transforms",
                   type=str, help="service name for robot state publisher")

    p.add_argument("--offset", default=[0,0,0,0,0,0],
                   type=float, nargs='+', help="offset to room origin from /map frame (rpy or quat)")

    args = p.parse_args()
    if len(args.offset) < 6 or len(args.offset) > 7:
        print len(args.offset)
        p.print_usage()
        sys.exit(1)
    sys.exit(exec_command(args))
