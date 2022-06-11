#!/usr/bin/env python

import rospy
import tf2_ros

from pyquaternion import Quaternion
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

import math
import sys


class T265OdometryTransformer(object):

    def __init__(self):

        rospy.init_node('t265_odometry_transformer')

        self._base_frame_id = str(
            rospy.get_param('~base_frame_id', 'base_link'))
        self._odom_frame_id = str(rospy.get_param('~odom_frame_id', 'odom'))
        self._translation_base_link_to_pose_frame_x = float(
            rospy.get_param('~translation_base_link_to_pose_frame_x'))
        self._translation_base_link_to_pose_frame_y = float(
            rospy.get_param('~translation_base_link_to_pose_frame_y'))
        self._translation_base_link_to_pose_frame_z = float(
            rospy.get_param('~translation_base_link_to_pose_frame_z'))
        self._rotation_base_link_to_pose_frame_x = float(
            rospy.get_param('~rotation_base_link_to_pose_frame_x'))
        self._rotation_base_link_to_pose_frame_y = float(
            rospy.get_param('~rotation_base_link_to_pose_frame_y'))
        self._rotation_base_link_to_pose_frame_z = float(
            rospy.get_param('~rotation_base_link_to_pose_frame_z'))
        self._rotation_base_link_to_pose_frame_w = float(
            rospy.get_param('~rotation_base_link_to_pose_frame_w'))

        self._publish_tf = bool(rospy.get_param('~publish_tf', True))
        self._2d_mode = bool(rospy.get_param('~2d_mode', True))

        self._quat_base_to_t265 = Quaternion(
            self._rotation_base_link_to_pose_frame_w,
            self._rotation_base_link_to_pose_frame_x,
            self._rotation_base_link_to_pose_frame_y,
            self._rotation_base_link_to_pose_frame_z
        ).normalised
        self._pos_base_to_t265 = np.array([
            self._translation_base_link_to_pose_frame_x,
            self._translation_base_link_to_pose_frame_y,
            self._translation_base_link_to_pose_frame_z
        ])

        if self._quat_base_to_t265.norm < 0.1:
            rospy.logerr('Rotation of base_link to pose_frame is invalid.')
            sys.exit(1)

        self._quat_t265_to_base = self._quat_base_to_t265.inverse
        self._pos_t265_to_base = self._quat_t265_to_base.rotate(
            -self._pos_base_to_t265)

        self._tf_br = tf2_ros.TransformBroadcaster()
        self._pub_odom = rospy.Publisher('~odom_out', Odometry, queue_size=1)
        self._sub_odom = rospy.Subscriber(
            '~odom_in', Odometry, self.cb_odometry)

    def cb_odometry(self, msg):

        # Nan check
        if (math.isnan(msg.pose.pose.position.x) or
            math.isnan(msg.pose.pose.position.y) or
            math.isnan(msg.pose.pose.position.z) or
            math.isnan(msg.pose.pose.orientation.x) or
            math.isnan(msg.pose.pose.orientation.y) or
            math.isnan(msg.pose.pose.orientation.z) or
            math.isnan(msg.twist.twist.linear.x) or
            math.isnan(msg.twist.twist.linear.y) or
            math.isnan(msg.twist.twist.linear.z) or
            math.isnan(msg.twist.twist.angular.x) or
            math.isnan(msg.twist.twist.angular.y) or
                math.isnan(msg.twist.twist.angular.z)):
            rospy.logwarn_throttle(
                10, 'Received an odom message with nan values')
            return

        pos_odom_t265_to_t265 = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        quat_odom_t265_to_t265 = Quaternion(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ).normalised

        pos_odom_t265_to_base = pos_odom_t265_to_t265 + quat_odom_t265_to_t265.rotate(self._pos_t265_to_base)
        quat_odom_t265_to_base = quat_odom_t265_to_t265 * self._quat_t265_to_base

        pos_odom_base_to_odom_t265 = self._pos_base_to_t265
        quat_odom_base_to_odom_t265 = self._quat_base_to_t265
        pos_odom_base_to_base = pos_odom_base_to_odom_t265 + quat_odom_base_to_odom_t265.rotate(pos_odom_t265_to_base)
        quat_odom_base_to_base = quat_odom_base_to_odom_t265 * quat_odom_t265_to_base

        vel_pos_t265_on_t265 = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        vel_rot_t265_on_t265 = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])
        vel_pos_base_on_base = self._quat_base_to_t265.rotate(
            vel_pos_t265_on_t265 +
            np.cross(vel_rot_t265_on_t265, self._pos_t265_to_base)
        )
        vel_rot_base_on_base = self._quat_base_to_t265.rotate(
            vel_rot_t265_on_t265)

        pub_msg = Odometry()
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.header.frame_id = self._odom_frame_id
        pub_msg.child_frame_id = self._base_frame_id
        if self._2d_mode:
            quat_odom_base_to_base_2d = Quaternion(
                quat_odom_base_to_base.w,
                0,
                0,
                quat_odom_base_to_base.z
            ).normalised
            pub_msg.pose.pose.position.x = pos_odom_base_to_base[0]
            pub_msg.pose.pose.position.y = pos_odom_base_to_base[1]
            pub_msg.pose.pose.position.z = 0
            pub_msg.pose.pose.orientation.x = quat_odom_base_to_base_2d.x
            pub_msg.pose.pose.orientation.y = quat_odom_base_to_base_2d.y
            pub_msg.pose.pose.orientation.z = quat_odom_base_to_base_2d.z
            pub_msg.pose.pose.orientation.w = quat_odom_base_to_base_2d.w
            pub_msg.twist.twist.linear.x = vel_pos_base_on_base[0]
            pub_msg.twist.twist.linear.y = vel_pos_base_on_base[1]
            pub_msg.twist.twist.linear.z = 0
            pub_msg.twist.twist.angular.x = 0
            pub_msg.twist.twist.angular.y = 0
            pub_msg.twist.twist.angular.z = vel_rot_base_on_base[2]
        else:
            pub_msg.pose.pose.position.x = pos_odom_base_to_base[0]
            pub_msg.pose.pose.position.y = pos_odom_base_to_base[1]
            pub_msg.pose.pose.position.z = pos_odom_base_to_base[2]
            pub_msg.pose.pose.orientation.x = quat_odom_base_to_base.x
            pub_msg.pose.pose.orientation.y = quat_odom_base_to_base.y
            pub_msg.pose.pose.orientation.z = quat_odom_base_to_base.z
            pub_msg.pose.pose.orientation.w = quat_odom_base_to_base.w
            pub_msg.twist.twist.linear.x = vel_pos_base_on_base[0]
            pub_msg.twist.twist.linear.y = vel_pos_base_on_base[1]
            pub_msg.twist.twist.linear.z = vel_pos_base_on_base[2]
            pub_msg.twist.twist.angular.x = vel_rot_base_on_base[0]
            pub_msg.twist.twist.angular.y = vel_rot_base_on_base[1]
            pub_msg.twist.twist.angular.z = vel_rot_base_on_base[2]
        # Covariance is not transformed currently
        pub_msg.pose.covariance = msg.pose.covariance
        pub_msg.twist.covariance = msg.twist.covariance
        self._pub_odom.publish(pub_msg)

        if self._publish_tf:

            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = pub_msg.header.frame_id
            t.child_frame_id = pub_msg.child_frame_id
            t.transform.translation.x = pub_msg.pose.pose.position.x
            t.transform.translation.y = pub_msg.pose.pose.position.y
            t.transform.translation.z = pub_msg.pose.pose.position.z
            t.transform.rotation.x = pub_msg.pose.pose.orientation.x
            t.transform.rotation.y = pub_msg.pose.pose.orientation.y
            t.transform.rotation.z = pub_msg.pose.pose.orientation.z
            t.transform.rotation.w = pub_msg.pose.pose.orientation.w
            self._tf_br.sendTransform(t)


def main():

    a = T265OdometryTransformer()
    rospy.spin()


if __name__ == '__main__':
    main()
