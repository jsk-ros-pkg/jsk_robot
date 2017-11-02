#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import argparse
import rospy
import rostopic
from logger_base import LoggerBase


class MongoRecord(LoggerBase):
    def __init__(self, argv=None):
        if argv is None:
            self.topics = rospy.get_param("~topics")
            self.blocking = rospy.get_param("~blocking", False)
            self.queue_size = rospy.get_param("~queue_size", 1)
        else:
            args = self.parse_args(argv)
            self.topics = args.topics
            self.blocking = args.blocking
            self.queue_size = args.queue_size

        self.subscribers = {}

        LoggerBase.__init__(self)

    def parse_args(self, argv):
        p = argparse.ArgumentParser()
        p.add_argument("topics", nargs="+", type=str,
                       help="Topic name to record")
        p.add_argument("-b", "--blocking", action="store_true",
                       help="Enable blocking on each message")
        p.add_argument("-q", "--queue-size", type=int,
                       help="Queue size of each subscriber",
                       default=1)
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
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self.check_topic()
            self.spinOnce()
            rate.sleep()


if __name__ == '__main__':
    pass
