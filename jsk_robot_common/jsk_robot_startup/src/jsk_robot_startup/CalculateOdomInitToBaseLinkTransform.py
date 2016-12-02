#! /usr/bin/env python

import rospy
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, TransformStamped
import tf
import threading
from odometry_utils import make_homogeneous_matrix

class CalculateOdomInitToBaseLinkTransform(object):
    def __init__(self):
        rospy.init_node("CalculateOdomInitToBaseLinkTransform", anonymous=True)
        self.offset_transform = None
        self.base_odom = None
        self.lock = threading.Lock()
        self.odom_frame = rospy.get_param("~init_odom_frame", "odom_init")
        self.body_frame = rospy.get_param("~body_link_frame", "BODY")
        # execute rate
        self.rate = float(rospy.get_param("~rate", 10))
        self.r = rospy.Rate(self.rate)
        self.pub = rospy.Publisher("~output", TransformStamped, queue_size = 1, latch = True)
        self.base_odom_sub = rospy.Subscriber("~base_odom", Odometry, self.base_odom_callback)
        self.base_to_init_transform_sub = rospy.Subscriber("~base_to_init_transform", TransformStamped, self.base_to_init_transform_callback) # init_transform is assumed to be transform of base_odom -> init_odom

    def calculate_init_to_base_link_transform(self, base_odom, base_to_init_transform, stamp):
        # tf relationships: init_odom <- base_odom -> base_link
        # offset_odom is init_odom -> base_link
        # TODO: check timestamps of base_odom and source_odom and fix if necessary
        if base_odom == None or  base_to_init_transform == None:
            return None
        base_odom_matrix = make_homogeneous_matrix([getattr(self.base_odom.pose.pose.position, attr) for attr in ["x", "y", "z"]], [getattr(self.base_odom.pose.pose.orientation, attr) for attr in ["x", "y", "z", "w"]]) # base_odom -> body_link
        offset_matrix = numpy.linalg.inv(base_to_init_transform).dot(base_odom_matrix)
        trans = list(offset_matrix[:3, 3])
        rot = list(tf.transformations.quaternion_from_matrix(offset_matrix))
        offset_transform = TransformStamped()
        offset_transform.header.stamp = stamp
        offset_transform.header.frame_id = self.odom_frame
        offset_transform.child_frame_id = self.body_frame
        offset_transform.transform.translation = Vector3(*trans)
        offset_transform.transform.rotation = Quaternion(*rot)
        return offset_transform

    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def base_to_init_transform_callback(self, msg):
        with self.lock:
            base_to_init_transform = make_homogeneous_matrix([getattr(msg.transform.translation, attr) for attr in ["x", "y", "z"]], [getattr(msg.transform.rotation, attr) for attr in ["x", "y", "z", "w"]]) # base_odom -> init_odom
            self.offset_transform = self.calculate_init_to_base_link_transform(self.base_odom, base_to_init_transform, msg.header.stamp)
            if self.offset_transform != None:
                self.pub.publish(self.offset_transform)

    def base_odom_callback(self, msg):
        with self.lock:
            self.base_odom = msg
            if not self.offset_transform:
                # offset is not initialized
                base_to_init_transform = make_homogeneous_matrix([0, 0, 0], [0, 0, 0, 1])
                self.offset_transform = self.calculate_init_to_base_link_transform(self.base_odom, base_to_init_transform, msg.header.stamp)
                if self.offset_transform != None:                
                    self.pub.publish(self.offset_transform)
