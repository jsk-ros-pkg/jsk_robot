#!/usr/bin/env python
# -*- coding: utf-8 -*-

import PyKDL
import actionlib
import rospy
import tf2_ros
import tf2_geometry_msgs

from jsk_recognition_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool

def convert_msg_point_to_kdl_vector(point):
    return PyKDL.Vector(point.x,point.y,point.z)

class PersonTracker(object):

    def __init__(self):

        self._frame_id_robot = rospy.get_param('~frame_id_robot','body')
        self._label_person = rospy.get_param('~label_person',0)
        self._dist_visible = rospy.get_param('~dist_visible',1.0)
        self._timeout_transform = rospy.get_param('~timeout_transform',0.05)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._sub_bbox_array = rospy.Subscriber(
                '~bbox_array',
                BoundingBoxArray,
                self._cb_bbox_array
                )
        self._pub_visible = rospy.Publisher(
                '~visible',
                Bool,
                queue_size=1
                )
        self._pub_people_pose_array = rospy.Publisher(
                '~people_pose_array',
                PoseArray,
                queue_size=1
                )

    def _cb_bbox_array(self,msg):

        frame_id_msg = msg.header.frame_id
        frame_id_robot = self._frame_id_robot

        try:
            frame_fixed_to_robot = tf2_geometry_msgs.transform_to_kdl(
                self._tf_buffer.lookup_transform(
                    frame_id_robot,
                    frame_id_msg,
                    msg.header.stamp,
                    timeout=rospy.Duration(self._timeout_transform)
                )
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr('lookup transform failed. {}'.format(e))
            return

        # check if there is a person
        is_person_visible = False
        people_pose_array = PoseArray()
        people_pose_array.header.stamp = msg.header.stamp
        people_pose_array.header.frame_id = frame_id_robot
        for bbox in msg.boxes:
            if bbox.label == self._label_person:
                vector_bbox = convert_msg_point_to_kdl_vector(bbox.pose.position)
                vector_bbox_robotbased = frame_fixed_to_robot * vector_bbox
                rospy.logdebug('Person found at the distance {}'.format(vector_bbox_robotbased.Norm()))
                if vector_bbox_robotbased.Norm() < self._dist_visible:
                    pose = Pose()
                    pose.position.x = vector_bbox_robotbased[0]
                    pose.position.y = vector_bbox_robotbased[1]
                    pose.position.z = vector_bbox_robotbased[2]
                    pose.orientation.w = 1.0
                    people_pose_array.poses.append(pose)
                    is_person_visible = True
                    break
        self._pub_people_pose_array.publish(people_pose_array)
        self._pub_visible.publish(Bool(is_person_visible))

def main():

    rospy.init_node('person_tracker')
    person_tracker = PersonTracker()
    rospy.spin()

if __name__=='__main__':
    main()
