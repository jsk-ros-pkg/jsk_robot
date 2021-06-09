#!/usr/bin/env python

import rospy
import tf2_ros

from pyquaternion import Quaternion
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

import math

class OdometryTransformer(object):

    def __init__(self):

        rospy.init_node('odometry_transformer')

        self._timeout = float(rospy.get_param('~timeout', 5.0))

        self._frame_id_base_link = str(
            rospy.get_param('~frame_id_base_link', 'base_link'))
        self._frame_id_odom = str(rospy.get_param('~frame_id_odom', 'odom'))
        self._translation_base_link_to_pose_frame_x = float(rospy.get_param('~translation_base_link_to_pose_frame_x'))
        self._translation_base_link_to_pose_frame_y = float(rospy.get_param('~translation_base_link_to_pose_frame_y'))
        self._translation_base_link_to_pose_frame_z = float(rospy.get_param('~translation_base_link_to_pose_frame_z'))
        self._rotation_base_link_to_pose_frame_x = float(rospy.get_param('~rotation_base_link_to_pose_frame_x'))
        self._rotation_base_link_to_pose_frame_y = float(rospy.get_param('~rotation_base_link_to_pose_frame_y'))
        self._rotation_base_link_to_pose_frame_z = float(rospy.get_param('~rotation_base_link_to_pose_frame_z'))
        self._rotation_base_link_to_pose_frame_w = float(rospy.get_param('~rotation_base_link_to_pose_frame_w'))

        self._publish_tf = bool(rospy.get_param('~publish_tf', True))
        self._2d_mode = bool(rospy.get_param('~2d_mode', True))

        self._quat_t265_basebased = Quaternion(
            self._rotation_base_link_to_pose_frame_w,
            self._rotation_base_link_to_pose_frame_x,
            self._rotation_base_link_to_pose_frame_y,
            self._rotation_base_link_to_pose_frame_z
        ).normalised
        self._pos_t265_basebased = np.array([
            self._translation_base_link_to_pose_frame_x,
            self._translation_base_link_to_pose_frame_y,
            self._translation_base_link_to_pose_frame_z
        ])
        self._quat_base_t265based = self._quat_t265_basebased.inverse
        self._pos_base_t265based = self._quat_base_t265based.rotate(-self._pos_t265_basebased)

        self._tf_br = tf2_ros.TransformBroadcaster()
        self._pub_odom = rospy.Publisher('~odom_out', Odometry, queue_size=1)
        self._sub_odom = rospy.Subscriber(
            '~odom_in', Odometry, self.cb_odometry)

    def cb_odometry(self, msg):

        # Nan check
        if math.isnan( msg.pose.pose.position.x ) or \
           math.isnan( msg.pose.pose.position.y ) or \
           math.isnan( msg.pose.pose.position.z ) or \
           math.isnan( msg.pose.pose.orientation.x ) or \
           math.isnan( msg.pose.pose.orientation.y ) or \
           math.isnan( msg.pose.pose.orientation.z ) or \
           math.isnan( msg.twist.twist.linear.x ) or \
           math.isnan( msg.twist.twist.linear.y ) or \
           math.isnan( msg.twist.twist.linear.z ) or \
           math.isnan( msg.twist.twist.angular.x ) or \
           math.isnan( msg.twist.twist.angular.y ) or \
           math.isnan( msg.twist.twist.angular.z ):
            rospy.logwarn('Recieved an odom message with nan values')
            return

        pos_t265_odombased = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        quat_t265_odombased = Quaternion(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ).normalised

        pos_base_odombased = pos_t265_odombased + \
            quat_t265_odombased.rotate(self._pos_base_t265based)
        quat_base_odombased = quat_t265_odombased * self._quat_base_t265based

        pos_dot_t265_t265based = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        omega_t265_t265based = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])
        pos_dot_base_basebased = self._quat_t265_basebased.rotate(
            pos_dot_t265_t265based +
            np.cross(omega_t265_t265based, self._pos_base_t265based)
        )
        omega_base_basebased = self._quat_t265_basebased.rotate(
            omega_t265_t265based)

        pub_msg = Odometry()
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.header.frame_id = self._frame_id_odom
        pub_msg.child_frame_id = self._frame_id_base_link
        if self._2d_mode:
            quat_base_odombased_2d = Quaternion(
                quat_base_odombased.w,
                0,
                0,
                quat_base_odombased.z
            ).normalised
            pub_msg.pose.pose.position.x = pos_base_odombased[0]
            pub_msg.pose.pose.position.y = pos_base_odombased[1]
            pub_msg.pose.pose.position.z = 0
            pub_msg.pose.pose.orientation.x = quat_base_odombased_2d.x
            pub_msg.pose.pose.orientation.y = quat_base_odombased_2d.y
            pub_msg.pose.pose.orientation.z = quat_base_odombased_2d.z
            pub_msg.pose.pose.orientation.w = quat_base_odombased_2d.w
            pub_msg.twist.twist.linear.x = pos_dot_base_basebased[0]
            pub_msg.twist.twist.linear.y = pos_dot_base_basebased[1]
            pub_msg.twist.twist.linear.z = 0
            pub_msg.twist.twist.angular.x = 0
            pub_msg.twist.twist.angular.y = 0
            pub_msg.twist.twist.angular.z = omega_base_basebased[2]
        else:
            pub_msg.pose.pose.position.x = pos_base_odombased[0]
            pub_msg.pose.pose.position.y = pos_base_odombased[1]
            pub_msg.pose.pose.position.z = pos_base_odombased[2]
            pub_msg.pose.pose.orientation.x = quat_base_odombased.x
            pub_msg.pose.pose.orientation.y = quat_base_odombased.y
            pub_msg.pose.pose.orientation.z = quat_base_odombased.z
            pub_msg.pose.pose.orientation.w = quat_base_odombased.w
            pub_msg.twist.twist.linear.x = pos_dot_base_basebased[0]
            pub_msg.twist.twist.linear.y = pos_dot_base_basebased[1]
            pub_msg.twist.twist.linear.z = pos_dot_base_basebased[2]
            pub_msg.twist.twist.angular.x = omega_base_basebased[0]
            pub_msg.twist.twist.angular.y = omega_base_basebased[1]
            pub_msg.twist.twist.angular.z = omega_base_basebased[2]
        # TODO: tranform covariance
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

    a = OdometryTransformer()
    rospy.spin()


if __name__ == '__main__':
    main()
