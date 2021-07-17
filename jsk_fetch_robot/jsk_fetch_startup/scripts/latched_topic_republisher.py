#!/usr/bin/env python

import rosbag
import rospy


class LatchedTopicRepublisher(object):
    def __init__(self):
        self.topicname = rospy.get_param('~topicname', 'tf_static')
        self.bagfilename = rospy.get_param("~file")
        try:
            with rosbag.Bag(self.bagfilename, 'r') as inputbag:
                topic, msg, t = inputbag.read_messages(self.topicname).next()
            packagename, typename = msg._type.split('/')
            exec('from {}.msg import {}'.format(packagename, typename))

            self.publisher = rospy.Publisher(
                self.topicname, eval(typename), latch=True, queue_size=10)
            list_messages = []
            with rosbag.Bag(self.bagfilename, 'r') as inputbag:
                for topic, msg, t in inputbag.read_messages(self.topicname):
                    list_messages.append(msg)
            for message in list_messages:
                self.publisher.publish(message)
            rospy.loginfo('Republishing topic: {}'.format(self.topicname))
        except StopIteration:
            rospy.logwarn('Fail to republish topic: {}'.format(self.topicname))
            rospy.logwarn('There is no topic {} in {}'.format(
                self.topicname, self.bagfilename))


if __name__ == '__main__':
    rospy.init_node('latched_topic_republisher')
    app = LatchedTopicRepublisher()
    rospy.spin()
