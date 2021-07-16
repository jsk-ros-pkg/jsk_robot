#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import argparse
import re
import rospy
import rostopic
from .logger_base import LoggerBase

REGEX = re.compile(r"\$\{param\s*(\S+)\s*\}")


class MongoRecord(LoggerBase):
    def __init__(self, argv=None):
        subst_param = False
        if argv is None:
            self.topics = rospy.get_param("~topics")
            self.blocking = rospy.get_param("~blocking", False)
            self.queue_size = rospy.get_param("~queue_size", 1)
            self.update_rate = rospy.get_param("~update_rate", 1.0)
            subst_param = rospy.get_param("~subst_param", False)
            collection = rospy.get_param("~collection", None)
        else:
            args = self.parse_args(argv)
            self.topics = args.topics
            self.blocking = args.blocking
            self.queue_size = args.queue_size
            self.update_rate = args.update_rate
            subst_param = args.subst_param
            collection = args.collection
        self.subscribers = {}

        if subst_param:
            topics = []
            for t in self.topics:
                keys = REGEX.findall(t)
                splitted = REGEX.split(t)
                for i in range(len(splitted)):
                    if splitted[i] in keys:
                        splitted[i] = rospy.get_param(splitted[i])
                topics.append(str().join(splitted))
            self.topics = topics

        LoggerBase.__init__(self, col_name=collection)

    def parse_args(self, argv):
        p = argparse.ArgumentParser()
        p.add_argument("topics", nargs="+", type=str,
                       help="Topic name to record")
        p.add_argument("-b", "--blocking", action="store_true",
                       help="Enable blocking on each message")
        p.add_argument("-q", "--queue-size", type=int,
                       help="Queue size of each subscriber",
                       default=1)
        p.add_argument("-s", "--subst-param", action="store_true",
                       help="Enable substring param (e,g, '$(param robot/name)/list')")
        p.add_argument("-r", "--update-rate", type=float, default=1.0,
                       help="Update rate for checking topics")
        p.add_argument("-c", "--collection", type=str, default=None,
                       help="Collection name to record data")
        return p.parse_args(argv)

    def check_topic(self):
        for topic_name in self.topics:
            if topic_name not in self.subscribers:
                cls, _, _ = rostopic.get_topic_class(topic_name, blocking=False)
                if not cls: continue

                self.subscribers.update({
                    topic_name: rospy.Subscriber(topic_name, cls,
                                                 self.callback,
                                                 callback_args=topic_name,
                                                 queue_size=self.queue_size)
                })
                rospy.loginfo("Subscribed %s" % topic_name)

    def callback(self, msg, topic):
        self.insert(msg,
                    meta={'input_topic': topic},
                    wait=self.blocking)

    def run(self):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.check_topic()
            self.spinOnce()
            rate.sleep()


if __name__ == '__main__':
    import sys
    rospy.init_node("mongo_record", anonymous=True)

    if len(sys.argv) != len(rospy.myargv()):
        # spawn by roslaunch
        rec = MongoRecord()
    else:
        # spawn by command
        rec = MongoRecord(sys.argv[1:])

    rec.run()

