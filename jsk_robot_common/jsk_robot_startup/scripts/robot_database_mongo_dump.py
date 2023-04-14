#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copied from https://github.com/strands-project/mongodb_store/blob/melodic-devel/mongodb_store/scripts/mongodb_play.py
#
from __future__ import print_function

import argparse
import calendar
import cv2
import datetime
import json
import mongodb_store.util as mg_util
import os
import pymongo
import rospy
import sys
import yaml

# https://answers.ros.org/question/196365/is-there-a-general-way-to-convert-ros-messages-into-json-format/
import rosbridge_library.internal.message_conversion
from rosbridge_library.util import string_types, bson
rosbridge_library.internal.message_conversion.bson_only_mode = False
rosbridge_library.internal.message_conversion.binary_encoder = bson.Binary

# https://stackoverflow.com/questions/10971033/backporting-python-3-openencoding-utf-8-to-python-2
from io import open


from cv_bridge import CvBridge
from dateutil import tz
from sensor_msgs.msg import Image, CompressedImage

UTC = tz.gettz('UTC')
JST = tz.gettz('Asia/Tokyo')

MongoClient = mg_util.import_MongoClient()

TIME_KEY = '_meta.inserted_at'

def max_time(collection):
    return collection.find_one(sort=[(TIME_KEY, pymongo.DESCENDING)])['_meta']['inserted_at']

def min_time(collection):
    return collection.find_one(sort=[(TIME_KEY, pymongo.ASCENDING)])['_meta']['inserted_at']

def to_ros_time(dt):
    return rospy.Time(calendar.timegm(dt.utctimetuple()), dt.microsecond * 1000)

def to_datetime(rt):
    return datetime.datetime.utcfromtimestamp(rt.secs) + datetime.timedelta(microseconds     = rt.nsecs / 1000)

def ros_time_strftime(rt, format):
    """ converts a ros time to a datetime and calls strftime on it with the given format """
    return to_datetime(rt).strftime(format)

def mkdatetime(date_string):
    return datetime.datetime.strptime(date_string, '%Y-%m-%d %H:%M:%S')

def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="musca.jsk.imi.i.u-tokyo.ac.jp")
    parser.add_argument("--port", "-p", default=27017)
    parser.add_argument("--database", "-d", default="jsk_robot_lifelog")
    parser.add_argument("--collection", "-c", default="basil")
    parser.add_argument("--start", "-s", default="", help='start datetime of query, defaults to the earliest date stored in db, across all requested collections. Formatted "Y-m-d H:M" e.g. "2022-07-14 06:38:00"')
    parser.add_argument("--end", "-e", default="", help='end datetime of query, defaults to the latest date stored in db, across all requested collections. Formatted "Y-m-d H:M" e.g. "2022-07-14 06:39:00"')
    args = parser.parse_args()
    bridge = CvBridge()

    print("Connecting ... {}:{}".format(args.host, args.port))
    mongo_client=MongoClient(args.host, args.port)
    collection = mongo_client[args.database][args.collection]
    print("Selected  ... {}/{}".format(args.database,args.collection))

    # make sure there's an index on time in the collection so the sort operation doesn't require the whole collection to be loaded
    collection.ensure_index(TIME_KEY)

    # get the min and max time across all collections, conver to ros time
    if args.start:
        start_time = mkdatetime(args.start).replace(tzinfo=JST)
    else:
        start_time = min_time(collection).replace(tzinfo=UTC).astimezone(JST)

    if args.end:
        end_time = mkdatetime(args.end).replace(tzinfo=JST)
    else:
        end_time = max_time(collection).replace(tzinfo=UTC).astimezone(JST)

    print("  From : {}".format(start_time.strftime('%Y-%m-%d %H:%M:%S')))
    print("  To   : {}".format(end_time.strftime('%Y-%m-%d %H:%M:%S')))

    documents = collection.find({TIME_KEY: { '$gte': start_time, '$lte': end_time}}, sort=[(TIME_KEY, pymongo.ASCENDING)])
    documents_count = documents.count()
    print("This document contains {} messages".format(documents_count))
    # print(documents[0]['_meta']['inserted_at'])
    # print(documents[documents_count-1]['_meta']['inserted_at'])

    dirname = collection.full_name + start_time.strftime('_%Y-%m-%d_%H:%M:%S_') + end_time.strftime('_%Y-%m-%d_%H:%M:%S')
    if not os.path.exists(dirname):
        os.mkdir(dirname)
    msg_classes = {}
    for d in documents:
        stored_class = d['_meta']['stored_class']
        input_topic = d['_meta']['input_topic']
        inserted_at = d['_meta']['inserted_at']
        ros_time = to_ros_time(inserted_at)
        if stored_class in msg_classes:
            msg_class = msg_classes[stored_class]
        else:
            try:
                msg_class = msg_classes[stored_class] = mg_util.load_class(stored_class)
            except ImportError as e:
                print(";;")
                print(";; ImportError: {}".format(e))
                print(";;")
                print(";; try install ros-{}-{}".format(os.environ['ROS_DISTRO'], stored_class.split('.')[0].replace('_','-')))
                print(";;")
                sys.exit(-1)
        message = mg_util.dictionary_to_message(d, msg_class)

        if type(message) == Image:
            filename = "{}{}.jpg".format(ros_time.to_nsec(), input_topic.replace('/','-'))
            image = bridge.imgmsg_to_cv2(message)
            cv2.imwrite(os.path.join(dirname, filename), image)
        elif type(message) == CompressedImage:
            filename = "{}{}.jpg".format(ros_time.to_nsec(), input_topic.replace('/','-'))
            image = bridge.compressed_imgmsg_to_cv2(message)
            cv2.imwrite(os.path.join(dirname, filename), image)
        else:
            filename = "{}{}.json".format(ros_time.to_nsec(), input_topic.replace('/','-'))
            with open(os.path.join(dirname, filename), "w", encoding="utf-8") as f:
                # f.write(yaml.dump(yaml.load(str(message)), allow_unicode=True))
                ##
                # data = yaml.load(str(message)) does not work because of containing ':' in value.
                # > yaml.scanner.ScannerError: mapping values are not allowed here
                # > in "<unicode string>", line 26, column 41: ... meters: b'{\n  "sentence-ending": "\\u305f\\u3088\\u306d", \n  " ...
                yaml_data = rosbridge_library.internal.message_conversion.extract_values(message)
                json_data = json.dumps(yaml_data, default=lambda o: o.decode('utf-8') if isinstance(o, bytes) else o, indent=4, ensure_ascii=False)
                try:
                    json_data = json_data.decode('utf-8')  # Python2 need to decode to use write
                except:
                    pass  # Python3 does not have decode()
                # convert bytes objects to strings before serializing to JSON
                f.write(json_data)
        print("Writing.. {} ({})".format(filename, inserted_at))


# processes load main so move init_node out
if __name__ == "__main__":
    main(sys.argv)
