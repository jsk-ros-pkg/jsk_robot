#! /usr/bin/env python

import rospy
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TwistWithCovariance
from std_msgs.msg import Float64
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
        # tf parameters
        self.publish_tf = rospy.get_param("~publish_tf", True)
        if self.publish_tf:
            self.invert_tf = rospy.get_param("~invert_tf", True)
            self.broadcast = tf.TransformBroadcaster()
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.camera_frame = rospy.get_param(
            "~camera_frame", "left_camera_optical_frame")
        self.odom_frame = rospy.get_param("~odom_frame", "viso_odom")
        self.tf_duration = rospy.get_param("~tf_duration", 1)
        self.listener = tf.TransformListener(True, rospy.Duration(10))
        self.initial_base_to_odom_transformation = None
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
            self.initial_base_to_odom_transformation = None

    def source_odom_callback(self, msg):
        with self.lock:
            # calculate camera transform
            current_camera_to_base = self.calculate_camera_to_base_transform(
                msg.header.stamp)
            if current_camera_to_base is None:
                return

            source_odom_matrix = make_homogeneous_matrix(
                    [msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     msg.pose.pose.position.z],
                    [msg.pose.pose.orientation.x,
                     msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z,
                     msg.pose.pose.orientation.w])

            if self.initial_base_to_odom_transformation is None:
                self.initial_base_to_odom_transformation = numpy.linalg.inv(
                    numpy.dot(source_odom_matrix, current_camera_to_base))

            # calculate offseted odometry
            current_odom_to_base = numpy.dot(
                source_odom_matrix, current_camera_to_base)
            odom_relative_base_transformation = numpy.dot(
                self.initial_base_to_odom_transformation, current_odom_to_base)
            new_odom_matrix = odom_relative_base_transformation

            # make odometry msg. twist is copied from source_odom
            new_odom = copy.deepcopy(msg)
            new_odom.header.frame_id = self.odom_frame
            new_odom.child_frame_id = self.base_link_frame
            new_odom.pose.pose.position = Point(*list(new_odom_matrix[:3, 3]))
            new_odom.pose.pose.orientation = Quaternion(
                *list(tf.transformations.quaternion_from_matrix(new_odom_matrix)))
            new_pose_cov_matrix = numpy.matrix(
                new_odom.pose.covariance).reshape(6, 6)
            rotation_matrix = odom_relative_base_transformation[:3, :3]
            new_pose_cov_matrix[:3, :3] = (rotation_matrix.T).dot(
                new_pose_cov_matrix[:3, :3].dot(rotation_matrix))
            new_pose_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(
                new_pose_cov_matrix[3:6, 3:6].dot(rotation_matrix))
            new_odom.pose.covariance = numpy.array(
                new_pose_cov_matrix).reshape(-1,).tolist()

            # publish
            self.pub.publish(new_odom)
            if self.publish_tf:
                broadcast_transform(self.broadcast, new_odom, self.invert_tf)

    def calculate_camera_to_base_transform(self, stamp):
        try:
            (trans, rot) = self.listener.lookupTransform(
                self.camera_frame, self.base_link_frame, stamp)
        except:
            try:
                # rospy.logwarn("[%s] failed to solve camera_to_base tf in %f. Use rospy.Time(0): %s to %s", rospy.get_name(), stamp.to_sec(), self.camera_frame, self.base_link_frame)
                (trans, rot) = self.listener.lookupTransform(
                    self.camera_frame, self.base_link_frame, rospy.Time(0))
            except:
                rospy.logwarn("[%s] failed to solve camera_to_base tf: %s to %s", rospy.get_name(
                ), self.camera_frame, self.base_link_frame)
                return None
        camera_to_base_link = make_homogeneous_matrix(
            trans, rot)  # camera -> base_link
        return camera_to_base_link


if __name__ == '__main__':
    try:
        node = CameraToBaseOffset()
        node.execute()
    except rospy.ROSInterruptException:
        pass
