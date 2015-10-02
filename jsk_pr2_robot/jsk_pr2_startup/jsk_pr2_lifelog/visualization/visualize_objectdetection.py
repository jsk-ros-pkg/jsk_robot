#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from mongodb_store.message_store import MessageStoreProxy
from posedetection_msgs.msg import Object6DPose
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, Pose
from datetime import datetime, timedelta
import tf

from transform_utils import TransformationUtils as T
from visualization_utils import VisualizationUtils as V

class DBPlay(object):
    def __init__(self):
        self.db_name = rospy.get_param('~db_name','jsk_pr2_lifelog')
        self.col_name = rospy.get_param('~col_name', 'pr1012')
        self.duration = rospy.get_param('~duration', 30)
        self.msg_store = MessageStoreProxy(database=self.db_name, collection=self.col_name)
        rospy.loginfo("connected to %s.%s" % (self.db_name, self.col_name))
        self.pub = rospy.Publisher('/object_detection_marker_array', MarkerArray)
        self.marker_count = 0

        objs = self.msg_store.query(type=Object6DPose._type,
                                    meta_query={"inserted_at": {
                                        "$gt": datetime.now() - timedelta(days=self.duration)
                                    }},
                                    sort_query=[("$natural", -1)])

        first_obj_meta = objs[0][1]

        trans = [tuple(self.msg_store.query(type=TransformStamped._type,
                                            meta_query={"inserted_at": {
                                              "$lt": first_obj_meta["inserted_at"]
                                            }},
                                            sort_query=[("$natural", -1)],
                                            single=True))]

        trans += self.msg_store.query(type=TransformStamped._type,
                                      meta_query={"inserted_at": {
                                        "$gt": first_obj_meta["inserted_at"]
                                      }},
                                      sort_query=[("$natural", -1)])

        j = 0
        m_arr = MarkerArray()
        for i in range(len(objs)):
            o,o_meta = objs[i]
            t,t_meta = trans[j]
            if o_meta["inserted_at"] > t_meta["inserted_at"]:
                j += 1
                t,t_meta = trans[j]
            ps = T.transformPoseWithTransformStamped(o.pose, t)

            m_arr.markers += V.poseStampedToLabeledSphereMarker([ps, o_meta], o.type)

        while not rospy.is_shutdown():
            self.pub.publish(m_arr)
            rospy.sleep(1.0)
            rospy.logdebug("publishing objectdetection_marker_array")


if __name__ == '__main__':
    rospy.init_node('visualize_objectdetection', log_level=rospy.DEBUG)
    o = DBPlay()
