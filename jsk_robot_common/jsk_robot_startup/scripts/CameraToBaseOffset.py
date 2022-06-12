#! /usr/bin/env python

import rospy
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty
import tf
import time
import threading
import copy
from jsk_robot_startup.odometry_utils import broadcast_transform
from jsk_robot_startup.odometry_utils import make_homogeneous_matrix


class CameraToBaseOffset(object):
    def __init__(self):
        rospy.init_node("CameraToBaseOffset", anonymous=True)

        # frame_id
        self.base_frame_id = rospy.get_param("~base_frame_id", "BODY")
        self.camera_frame_id = rospy.get_param(
            "~camera_frame_id", "left_camera_optical_frame")
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "viso_odom")

        # tf parameters
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.invert_tf = rospy.get_param("~invert_tf", True)

        # other members
        if self.publish_tf:
            self.broadcast = tf.TransformBroadcaster()
        self.listener = tf.TransformListener(True, rospy.Duration(10))
        self.htm_odom_base_to_odom_camera = None
        self.lock = threading.Lock()

        self.source_odom_sub = rospy.Subscriber(
            "~source_odom", Odometry, self.source_odom_callback)
        self.init_signal_sub = rospy.Subscriber(
            "~init_signal", Empty, self.init_signal_callback)
        self.pub = rospy.Publisher("~output", Odometry, queue_size=1)

    def execute(self):
        rospy.spin()

    def init_signal_callback(self, msg):
        time.sleep(1)  # wait to update odom_init frame
        with self.lock:
            self.htm_odom_base_to_odom_camera = None

    def source_odom_callback(self, msg):
        with self.lock:
            # calculate camera transform
            htm_camera_to_base = self.get_htm_camera_to_base(msg.header.stamp)
            if htm_camera_to_base is None:
                return

            # htm (Homogeneous Transformation Matrix)
            #   from odom frame of camera to current camera frame
            htm_odom_camera_to_camera = make_homogeneous_matrix(
                [msg.pose.pose.position.x,
                 msg.pose.pose.position.y,
                 msg.pose.pose.position.z],
                [msg.pose.pose.orientation.x,
                 msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z,
                 msg.pose.pose.orientation.w]
            )

            if self.htm_odom_base_to_odom_camera is None:
                self.htm_odom_base_to_odom_camera = numpy.linalg.inv(
                    numpy.dot(htm_odom_camera_to_camera, htm_camera_to_base))

            # calculate transformed pose
            htm_odom_camera_to_base = numpy.dot(
                htm_odom_camera_to_camera, htm_camera_to_base)
            htm_odom_base_to_base = numpy.dot(
                self.htm_odom_base_to_odom_camera, htm_odom_camera_to_base)

            # calculate transformed pose covariance
            raw_pose_cov_matrix = numpy.matrix(
                msg.pose.covariance).reshape(6, 6)
            rotation_matrix = htm_odom_base_to_base[:3, :3]
            transformed_pose_cov_matrix = copy.deepcopy(raw_pose_cov_matrix)
            transformed_pose_cov_matrix[:3, :3] = (rotation_matrix.T).dot(
                raw_pose_cov_matrix[:3, :3].dot(rotation_matrix))
            transformed_pose_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(
                raw_pose_cov_matrix[3:6, 3:6].dot(rotation_matrix))

            # calculate transformed twist
            #
            # NOTICE: Effect from 1st derivative of position and rotation
            #                   from camera to base is not considered.
            #
            twist_on_camera = numpy.array([[msg.twist.twist.linear.x],
                                           [msg.twist.twist.linear.y],
                                           [msg.twist.twist.linear.y],
                                           [msg.twist.twist.angular.x],
                                           [msg.twist.twist.angular.y],
                                           [msg.twist.twist.angular.y]])
            rotation_matrix_camera_to_base = htm_camera_to_base[:3, :3]
            pos_c2b = htm_camera_to_base[4, :3]
            twist_transform_matrix = numpy.zeros((4, 4))
            twist_transform_matrix[0:3, 0:3] = rotation_matrix_camera_to_base
            twist_transform_matrix[3:6, 3:6] = rotation_matrix_camera_to_base
            twist_transform_matrix[0:3, 3:6] = numpy.dot(
                numpy.array([[0, -pos_c2b[2],  pos_c2b[1]],
                             [pos_c2b[2],           0, -pos_c2b[0]],
                             [-pos_c2b[1],  pos_c2b[0],           0]]),
                rotation_matrix_camera_to_base
            )
            twist_on_base = numpy.dot(
                twist_transform_matrix,
                twist_on_camera)

            # calculate transformed twist covariance
            raw_twist_cov_matrix = numpy.matrix(
                msg.twist.covariance).reshape(6, 6)
            transformed_twist_cov_matrix = (twist_transform_matrix.T).dot(
                            raw_twist_cov_matrix.dot(
                                twist_transform_matrix))

            # make odometry msg.
            transformed_odom_msg = copy.deepcopy(msg)
            transformed_odom_msg.header.frame_id = self.odom_frame_id
            transformed_odom_msg.child_frame_id = self.base_frame_id
            transformed_odom_msg.pose.pose.position = Point(
                *list(htm_odom_base_to_base[:3, 3]))
            transformed_odom_msg.pose.pose.orientation = Quaternion(
                *list(tf.transformations.quaternion_from_matrix(
                            htm_odom_base_to_base)))
            transformed_odom_msg.pose.covariance = numpy.array(
                        transformed_pose_cov_matrix).reshape(-1,).tolist()
            transformed_odom_msg.twist.twist.linear = Vector3(
                *list(twist_on_base[:3]))
            transformed_odom_msg.twist.twist.angular = Vector3(
                *list(twist_on_base[3:6]))
            transformed_odom_msg.twist.covariance = numpy.array(
                        transformed_twist_cov_matrix).reshape(-1,).tolist()

            # publish
            self.pub.publish(transformed_odom_msg)
            if self.publish_tf:
                broadcast_transform(
                    self.broadcast, transformed_odom_msg, self.invert_tf)

    def get_htm_camera_to_base(self, stamp):
        try:
            (trans, rot) = self.listener.lookupTransform(
                self.camera_frame_id,
                self.base_frame_id,
                stamp)
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            try:
                rospy.logwarn(
                    "[%s] failed to solve camera_to_base tf in %f. Use rospy.Time(0): %s to %s",
                    rospy.get_name(),
                    stamp.to_sec(),
                    self.camera_frame_id,
                    self.base_frame_id)
                (trans, rot) = self.listener.lookupTransform(
                    self.camera_frame_id,
                    self.base_frame_id,
                    rospy.Time(0))
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                rospy.logwarn(
                    "[%s] failed to solve camera_to_base tf: %s to %s",
                    rospy.get_name(),
                    self.camera_frame_id,
                    self.base_frame_id)
                return None
        return make_homogeneous_matrix(trans, rot)


if __name__ == '__main__':
    try:
        node = CameraToBaseOffset()
        node.execute()
    except rospy.ROSInterruptException:
        pass
