#! /usr/bin/env python

import rospy
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, TransformStamped
import tf
import copy
import threading
from jsk_robot_startup.IIRFilter import IIRFilter
from jsk_robot_startup.odometry_utils import transform_quaternion_to_euler

class OdometryIIRFilter(object):
    def __init__(self):
        rospy.init_node("OdometryIIRFilter", anonymous=True)
        # parameters
        self.rate = float(rospy.get_param("~rate", 100))
        self.odom_frame = rospy.get_param("~odom_frame", "filtered_odom")
        self.filter_dim = rospy.get_param("~filter_dimension", 2)
        self.cutoff = min(float(rospy.get_param("~cutoff", 10)), self.rate / 2.0) # must be larger than nyquist frequency
        self.odom = None
        self.filtered_odom = None
        self.prev_rpy = None
        self.dt = 0.0
        self.r = rospy.Rate(self.rate)
        self.lock = threading.Lock()
        self.filters = []
        for i in range(6):
            self.filters.append(IIRFilter(self.filter_dim, self.cutoff / self.rate))
        # tf
        self.publish_tf = rospy.get_param("~publish_tf", True)
        if self.publish_tf:
            self.invert_tf = rospy.get_param("~invert_tf", True)
            self.broadcast = tf.TransformBroadcaster()
            self.listener = tf.TransformListener()
        # pub/sub
        self.pub = rospy.Publisher("~output", Odometry, queue_size=10)
        self.odom_sub = rospy.Subscriber("~source_odom", Odometry, self.source_odom_callback)
        self.init_transform_sub = rospy.Subscriber("~initial_base_link_transform", TransformStamped, self.init_transform_callback)

    def initialize_filter(self):
        with self.lock:
            self.odom = None
            self.filtered_odom = None
            self.prev_rpy = None
            self.filters = []
            for i in range(6):
                self.filters.append(IIRFilter(self.filter_dim, self.cutoff / self.rate))

    def execute(self):
        while not rospy.is_shutdown():
            self.update()
            self.r.sleep()
            
    def init_transform_callback(self, msg):
        self.initialize_filter()
            
    def source_odom_callback(self, msg):
        with self.lock:
            self.odom = msg

    def update(self):
        with self.lock:
            if self.odom == None:
                return
            current_euler = transform_quaternion_to_euler([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w], self.prev_rpy)
            current_6d_pose = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z] + current_euler
            for i in range(6):
                current_6d_pose[i] = self.filters[i].execute(current_6d_pose[i])
            self.filtered_odom = copy.deepcopy(self.odom)
            self.filtered_odom.header.frame_id = self.odom_frame
            self.filtered_odom.pose.pose.position = Point(*current_6d_pose[:3])
            self.filtered_odom.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(*current_6d_pose[3:6]))
            self.prev_rpy = current_6d_pose[3:6]
            self.pub.publish(self.filtered_odom)
            if self.publish_tf:
                broadcast_transform(self.broadcast, self.filtered_odom, self.invert_tf)
