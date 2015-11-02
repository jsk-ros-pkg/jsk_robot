#! /usr/bin/env python

import rospy
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Float64, Empty
import tf
import time
import threading
import copy

class OdometryOffset(object):
    def __init__(self):
        rospy.init_node("OdometryFeedbackWrapper", anonymous=True)
        self.rate = float(rospy.get_param("~rate", 100))
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.invert_tf = rospy.get_param("~invert_tf", True)
        self.odom_frame = rospy.get_param("~odom_frame", "offset_odom")
        self.base_odom_frame = rospy.get_param("~base_odom_frame", "odom_init")
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.tf_duration = rospy.get_param("~tf_duration", 1)
        self.broadcast = tf.TransformBroadcaster()
        self.listener = tf.TransformListener(True, rospy.Duration(120))
        self.r = rospy.Rate(self.rate)
        self.offset_matrix = None
        self.lock = threading.Lock()
        self.source_odom_sub = rospy.Subscriber("~source_odom", Odometry, self.source_odom_callback)
        self.init_signal_sub = rospy.Subscriber("~init_signal", Empty, self.init_signal_callback)
        self.pub = rospy.Publisher("~output", Odometry)        

    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def calculate_offset(self, odom):
        try:
            self.listener.waitForTransform(self.base_odom_frame, odom.child_frame_id, odom.header.stamp, rospy.Duration(self.tf_duration))
            (trans,rot) = self.listener.lookupTransform(self.base_odom_frame, odom.child_frame_id, odom.header.stamp)
        except:
            rospy.logwarn("[%s] failed to solve tf in initialize_odometry: %s to %s", rospy.get_name(), self.base_odom_frame, odom.child_frame_id)
            return None
        base_odom_to_base_link = self.make_homogeneous_matrix(trans, rot) # base_odom -> base_link
        base_link_to_odom = numpy.linalg.inv(self.make_homogeneous_matrix([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z],
                                                                          [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                                                           odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])) # base_link -> odom
        return base_odom_to_base_link.dot(base_link_to_odom) # base_odom -> odom
        
    def init_signal_callback(self, msg):
        time.sleep(1) # wait to update odom_init frame
        with self.lock:
            self.offset_matrix = None
            
    def source_odom_callback(self, msg):
        with self.lock:
            if self.offset_matrix != None:
                source_odom_matrix = self.make_homogeneous_matrix([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                                                                  [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                new_odom = copy.deepcopy(msg)
                new_odom.header.frame_id = self.odom_frame
                new_odom.child_frame_id = self.base_link_frame        
                    
                # offset coords
                new_odom_matrix = self.offset_matrix.dot(source_odom_matrix)
                new_odom.pose.pose.position = Point(*list(new_odom_matrix[:3, 3]))
                new_odom.pose.pose.orientation = Quaternion(*list(tf.transformations.quaternion_from_matrix(new_odom_matrix)))

                # offset covariance
                new_pose_cov_matrix = numpy.matrix(new_odom.pose.covariance).reshape(6, 6)
                rotation_matrix = self.offset_matrix[:3, :3]
                new_pose_cov_matrix[:3, :3] = (rotation_matrix.T).dot(new_pose_cov_matrix[:3, :3].dot(rotation_matrix))
                new_pose_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(new_pose_cov_matrix[3:6, 3:6].dot(rotation_matrix))
                new_odom.pose.covariance = numpy.array(new_pose_cov_matrix).reshape(-1,).tolist()

                # publish
                self.pub.publish(new_odom)
                if self.publish_tf:
                    self.broadcast_transform(new_odom)
            else:
                current_offset_matrix = self.calculate_offset(msg)
                if current_offset_matrix != None:
                    self.offset_matrix = current_offset_matrix

    def make_homogeneous_matrix(self, trans, rot):
        homogeneous_matrix = tf.transformations.quaternion_matrix(rot)
        homogeneous_matrix[:3, 3] = numpy.array(trans).reshape(1, 3)
        return homogeneous_matrix

    def broadcast_transform(self, odom):
        position = [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]
        orientation = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        if self.invert_tf:
            homogeneous_matrix = self.make_homogeneous_matrix(position, orientation)
            homogeneous_matrix_inv = numpy.linalg.inv(homogeneous_matrix)
            position = list(homogeneous_matrix_inv[:3, 3])
            orientation = list(tf.transformations.quaternion_from_matrix(homogeneous_matrix_inv))
            parent_frame = odom.child_frame_id
            target_frame = odom.header.frame_id
        else:
            parent_frame = odom.header.frame_id
            target_frame = odom.child_frame_id
        self.broadcast.sendTransform(position, orientation, odom.header.stamp, target_frame, parent_frame)
