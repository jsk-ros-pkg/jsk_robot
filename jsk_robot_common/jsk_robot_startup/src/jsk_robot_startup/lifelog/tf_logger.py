#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
import tf2_ros
import yaml
from tf2_msgs.srv import FrameGraph
from tf2_msgs.msg import TFMessage
from tf2_ros import ExtrapolationException
from logger_base import LoggerBase


class TFLogger(LoggerBase):
    def __init__(self):
        LoggerBase.__init__(self)

        self.log_rate = rospy.get_param('~log_rate', 1.0)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.wait_for_service("~tf2_frames")
        self.get_frames = rospy.ServiceProxy("~tf2_frames", FrameGraph)

        self.timer = rospy.Timer(rospy.Duration(self.log_rate), self.timer_callback)

    def timer_callback(self, event):
        time = event.last_real or event.current_real - rospy.Duration(self.log_rate)
        try:
            graph = yaml.load(self.get_frames().frame_yaml)
        except Exception as e:
            rospy.logerr(e)
            return

        transforms = []
        for child, info in graph.items():
            parent = info["parent"]
            try:
                transforms += [self.tf_buffer.lookup_transform(parent, child, time)]
            except ExtrapolationException:
                pass
        if not transforms: return

        try:
            self.insert(TFMessage(transforms))
        except Exception as e:
            rospy.logerr(e)

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            self.spinOnce()


if __name__ == '__main__':
    rospy.init_node("tf_logger")
    TFLogger().run()
