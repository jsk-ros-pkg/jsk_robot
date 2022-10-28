#!/usr/bin/env python
# -*- coding: utf-8 -*-

import base64
import numpy as np
import pickle
import rospy
import sys

# date
from datetime import datetime, timedelta, tzinfo
from dateutil import tz
import calendar
import pytz

# mongo
from mongodb_store.message_store import MessageStoreProxy
import mongodb_store.util as MU
import pymongo

# message
from jsk_robot_startup.lifelog.logger_base import LoggerBase
from sensor_msgs.msg import CompressedImage
from smach_msgs.msg import SmachContainerStatus

# image processing
from cv_bridge import CvBridge
import cv2

# global variabels
JST = tz.gettz('Asia/Tokyo')
bridge = CvBridge()

# sample functions
def query_latest_smach():
    try:
        rospy.loginfo("Loading last smach execution...")
        last_msg, _ = msg_store.query(
            SmachContainerStatus._type,
            {"info": "START"},
            single=True,
            sort_query=[("_meta.inserted_at", pymongo.DESCENDING)]
        )
        msgs = msg_store.query(
            SmachContainerStatus._type,
            {"header.stamp.secs": {"$gt": last_msg.header.stamp.secs}},
            sort_query=[("_meta.inserted_at", pymongo.ASCENDING)]
        )

        def status_to_img(msg):
            if sys.version_info.major < 3:
                local_data_str = pickle.loads(msg.local_data)
            else:
                local_data_str = pickle.loads(
                    msg.local_data.encode('utf-8'), encoding='utf-8')
            print("{} @{}".format(local_data_str['DESCRIPTION'],
                                  datetime.fromtimestamp(msg.header.stamp.to_sec(), JST)))
            imgmsg = None
            if 'IMAGE' in local_data_str and local_data_str['IMAGE']:
                imgmsg = CompressedImage()
                imgmsg.deserialize(base64.b64decode(local_data_str['IMAGE']))
            return imgmsg

        return filter(lambda x: x is not None, map(lambda x: status_to_img(x[0]), msgs))
    except Exception as e:
        rospy.logerr('failed to load images from db: %s' % e)
    return None


def query_images(now  = datetime.now(JST)-timedelta(hours=0),
                 then = datetime.now(JST)-timedelta(hours=1)):
    try:
        rospy.loginfo("Loading images...")
        msgs = msg_store.query(
            CompressedImage._type,
            {"_meta.inserted_at": {"$lt": now, "$gte": then}},
            sort_query=[("_meta.inserted_at", pymongo.ASCENDING)]
        )
        return map(lambda x: x[0], msgs)
    except Exception as e:
        rospy.logerr('failed to load images from db: %s' % e)
    return None

if __name__ == '__main__':
    rospy.init_node('sample_robot_database')
    db_name = 'jsk_robot_lifelog'
    col_name = 'basil' # pr1012, fetch17 etc..
    msg_store = MessageStoreProxy(database=db_name, collection=col_name)

    print("> sample program for robot database")
    print("> 1: get latest smach data")
    print("> 2: get last 1 hours image data")
    #key = int(input("> {1, 2, 3..} : "))
    key = 2

    if key == 1:
        msgs = query_latest_smach()
    elif key == 2:
        msgs = query_images(then = datetime(2022, 11,  1, 17,  0, tzinfo=JST),
                            now  = datetime(2022, 11,  1, 20, 10, tzinfo=JST))
    else:
        print("unknown inputs...")

    # show data..
    for msg in msgs:
        print(" @{}".format(datetime.fromtimestamp(msg.header.stamp.to_sec(), JST)))
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('image', cv_image)
        cv2.waitKey(50)
