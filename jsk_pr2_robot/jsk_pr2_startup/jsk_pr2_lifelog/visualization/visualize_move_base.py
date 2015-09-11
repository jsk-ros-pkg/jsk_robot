#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from mongodb_store.message_store import MessageStoreProxy
from posedetection_msgs.msg import Object6DPose
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped
from datetime import datetime, timedelta
import tf

from transform_utils import TransformationUtils as T
from visualization_utils import VisualizationUtils as V

class DBPlay(object):
    def __init__(self):
        self.db_name = rospy.get_param('~db_name','jsk_robot_lifelog')
        self.col_name = rospy.get_param('~col_name', 'pr1012')
        self.use_ros_time = rospy.get_param('~use_ros_time', True)
        self.use_sim_time = rospy.get_param('use_sim_time', False)
        self.downsample = rospy.get_param("~downsample", 3)
        self.label_downsample = rospy.get_param("~label_downsample", 10)
        self.duration = rospy.get_param('~duration', 10) # days
        self.msg_store = MessageStoreProxy(database=self.db_name,
                                           collection=self.col_name)
        rospy.loginfo("connected to %s.%s" % (self.db_name, self.col_name))
        self.pub = rospy.Publisher('/move_base_marker_array', MarkerArray)
        self.marker_count = 0

    def run(self):
        while not rospy.is_shutdown():
            rospy.logdebug("$gt: %d" % (rospy.Time.now().secs - self.duration * 60 * 60 * 24))
            if self.use_ros_time or self.use_sim_time:
                trans = self.msg_store.query(type=TransformStamped._type,
                                             message_query={"header.stamp.secs": {
                                                 "$lte": rospy.Time.now().secs,
                                                 "$gt": rospy.Time.now().secs - self.duration * 60 * 60 * 24
                                             }},
                                             sort_query=[("$natural", 1)])
            else:
                trans = self.msg_store.query(type=TransformStamped._type,
                                             meta_query={"inserted_at": {
                            "$gt": datetime.utcnow() - timedelta(days=self.duration)
                            }},
                                             sort_query=[("$natural", 1)])
            rospy.loginfo("found %d msgs" % len(trans))
            m_arr = MarkerArray()
            m_arr.markers = V.transformStampedArrayToLabeledLineStripMarker(trans[::self.downsample], label_downsample=self.label_downsample, discrete=True)
            m_arr.markers = V.transformStampedArrayToLabeledArrayMarker(trans[::self.downsample], label_downsample=self.label_downsample, discrete=True)
            self.pub.publish(m_arr)
            rospy.sleep(1.0)
            rospy.loginfo("publishing move_base_marker_array")


if __name__ == '__main__':
    rospy.init_node('visualize_move_base')
    o = DBPlay()
    o.run()
