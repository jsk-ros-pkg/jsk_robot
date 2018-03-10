#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Detail of visualization_msgs/msg/Marker.msg of Indigo and that of Kinetic are a little bit different, so checksums of them are different of course. That means on Kinetic you cannot subscribe the topic of which type is visualization_msgs/Marker published on  Indigo.
# Use this script on Kinetic when you want to subscribe the topic of which type is visualization_msgs/MarkerArray published on Indigo.
# By executing this script, you can subscribe /marker_msg_from_indigo_to_kinetic topic, the content of which is the same with visualization_msgs/MarkerArray published on Indigo.

import os
import rospy
from visualization_msgs.msg import Marker, MarkerArray


class VisualizationMarkerBridgeForKinetic(object):
    def __init__(self):
        if os.getenv("ROS_DISTRO", "kinetic") != "kinetic":
            rospy.logwarn("This node works only on kinetic")

        rate = rospy.get_param("~rate", 1.0)
        self.suffix = rospy.get_param("~suffix", "kinetic")

        self.publishers = dict()
        self.subscribers = dict()

        self.timer = rospy.Timer(rospy.Duration(rate), self.timer_cb)

    def msg_cb(self, msg, name):
        try:
            self.publishers[name].publish(msg)
            rospy.logdebug("Relayed %s" % name)
        except Exception as e:
            rospy.logerr(e)

    def timer_cb(self, event=None):
        all_topics = rospy.get_published_topics()
        markers = [t[0] for t in all_topics if t[1] == "visualization_msgs/Marker" and\
                   not t[0].endswith(self.suffix)]
        arrays = [t[0] for t in all_topics if t[1] == "visualization_msgs/MarkerArray" and\
                  not t[0].endswith(self.suffix)]
        names = markers + arrays

        for name in markers:
            if name not in self.publishers:
                new_name = name + "/" + self.suffix
                self.publishers[name] = rospy.Publisher(
                    new_name, Marker, queue_size=1)
                rospy.logdebug("Advertised %s" % new_name)

        for name in arrays:
            if name not in self.publishers:
                new_name = name + "/" + self.suffix
                self.publishers[name] = rospy.Publisher(
                    new_name, MarkerArray, queue_size=1)
                rospy.logdebug("Advertised %s" % new_name)

        for name, pub in self.publishers.items():
            # clean old topics
            if name not in names:
                try:
                    self.subscribers.pop(name).unregister()
                    rospy.logdebug("Removed sub %s" % name)
                except:
                    pass
                try:
                    self.publishers.pop(name).unregister()
                    rospy.logdebug("Removed pub %s" % name)
                except:
                    pass
            # subscribe topics subscribed
            elif pub.get_num_connections() > 0:
                if name not in self.subscribers:
                    self.subscribers[name] = rospy.Subscriber(
                        name, rospy.AnyMsg,
                        self.msg_cb, name)
                    rospy.logdebug("Subscribed %s" % name)
            # unsubscribe topics unsubscribed
            else:
                if name in self.subscribers:
                    self.subscribers.pop(name).unregister()
                    rospy.logdebug("Unsubscribed %s" % name)


if __name__ == '__main__':
    rospy.init_node("marker_msg_from_indigo_to_kinetic")
    m = VisualizationMarkerBridgeForKinetic()
    rospy.spin()
