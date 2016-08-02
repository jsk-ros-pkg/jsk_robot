#! /usr/bin/env python

import rospy
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3, TwistWithCovariance, TransformStamped
from std_msgs.msg import Float64, Empty
import tf
import time
import threading
import copy
from dynamic_reconfigure.server import Server
from jsk_robot_startup.cfg import OdometryOffsetReconfigureConfig
from odometry_utils import make_homogeneous_matrix, update_twist_covariance, update_pose, update_pose_covariance, broadcast_transform, transform_local_twist_to_global, transform_local_twist_covariance_to_global

class OdometryOffset(object):
    def __init__(self):
        rospy.init_node("OdometryOffset", anonymous=True)
        self.offset_matrix = None
        self.prev_odom = None
        self.initial_base_link_transform = make_homogeneous_matrix([0, 0, 0], [0, 0, 0, 1])
        self.lock = threading.Lock()
        # execute rate
        self.rate = float(rospy.get_param("~rate", 100))
        self.r = rospy.Rate(self.rate)
        # tf parameters
        self.publish_tf = rospy.get_param("~publish_tf", True)
        if self.publish_tf:
            self.broadcast = tf.TransformBroadcaster()
            self.invert_tf = rospy.get_param("~invert_tf", True)
        self.odom_frame = rospy.get_param("~odom_frame", "offset_odom")
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.tf_duration = rospy.get_param("~tf_duration", 1)
        # for filter (only used when use_twist_filter is True)
        self.use_twist_filter = rospy.get_param("~use_twist_filter", False)
        if self.use_twist_filter:
            self.filter_buffer_size = rospy.get_param("~filter_buffer_size", 5)
            self.filter_buffer = []
        # to overwrite probability density function (only used when overwrite_pdf is True)
        self.overwrite_pdf = rospy.get_param("~overwrite_pdf", False)
        if self.overwrite_pdf:
            self.twist_proportional_sigma = rospy.get_param("~twist_proportional_sigma", False)
            self.v_err_mean = [rospy.get_param("~mean_x", 0.0),
                               rospy.get_param("~mean_y", 0.0),
                               rospy.get_param("~mean_z", 0.0),
                               rospy.get_param("~mean_roll", 0.0),
                               rospy.get_param("~mean_pitch", 0.0),
                               rospy.get_param("~mean_yaw", 0.0)]
            self.v_err_sigma = [rospy.get_param("~sigma_x", 0.05),
                                rospy.get_param("~sigma_y", 0.1),
                                rospy.get_param("~sigma_z", 0.0001),
                                rospy.get_param("~sigma_roll", 0.0001),
                                rospy.get_param("~sigma_pitch", 0.0001),
                                rospy.get_param("~sigma_yaw", 0.01)]
            self.reconfigure_server = Server(OdometryOffsetReconfigureConfig, self.reconfigure_callback)
        self.source_odom_sub = rospy.Subscriber("~source_odom", Odometry, self.source_odom_callback)
        self.init_transform_sub = rospy.Subscriber("~initial_base_link_transform", TransformStamped, self.init_transform_callback) # init_transform is assumed to be transform of init_odom -> base_link
        self.pub = rospy.Publisher("~output", Odometry, queue_size = 1)

    def reconfigure_callback(self, config, level):
        with self.lock:
            for i, mean in enumerate(["mean_x", "mean_y", "mean_z", "mean_roll", "mean_pitch", "mean_yaw"]):
                self.v_err_mean[i] = config[mean]
            for i, sigma in enumerate(["sigma_x", "sigma_y", "sigma_z", "sigma_roll", "sigma_pitch", "sigma_yaw"]):
                self.v_err_sigma[i] = config[sigma]
        rospy.loginfo("[%s]" + "velocity mean updated: x: {0}, y: {1}, z: {2}, roll: {3}, pitch: {4}, yaw: {5}".format(*self.v_err_mean), rospy.get_name())                
        rospy.loginfo("[%s]" + "velocity sigma updated: x: {0}, y: {1}, z: {2}, roll: {3}, pitch: {4}, yaw: {5}".format(*self.v_err_sigma), rospy.get_name())
        return config

    def calculate_offset(self, source_odom):
        # tf relationships: init_odom -> source_odom -> body_link <- init_odom
        # offset_odom is init_odom -> source_odom
        # TODO: check timestamps of base_odom and source_odom and fix if necessary
        if self.initial_base_link_transform == None:
            rospy.loginfo("[%s] initial transform is not subscribed yet", rospy.get_name())
            return None
        source_odom_matrix = make_homogeneous_matrix([getattr(source_odom.pose.pose.position, attr) for attr in ["x", "y", "z"]], [getattr(source_odom.pose.pose.orientation, attr) for attr in ["x", "y", "z", "w"]]) # source_odom -> body_link
        return self.initial_base_link_transform.dot(numpy.linalg.inv(source_odom_matrix)) # T(init->src) * T(src->body) = T(init->body)

    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def init_transform_callback(self, msg):
        with self.lock:
            self.initial_base_link_transform = make_homogeneous_matrix([getattr(msg.transform.translation, attr) for attr in ["x", "y", "z"]], [getattr(msg.transform.rotation, attr) for attr in ["x", "y", "z", "w"]]) # base_odom -> init_odom
            self.prev_odom = None
            self.offset_matrix = None
            if self.use_twist_filter:
                self.filter_buffer = []

    def source_odom_callback(self, msg):
        with self.lock:
            if self.offset_matrix != None:
                source_odom_matrix = make_homogeneous_matrix([getattr(msg.pose.pose.position, attr) for attr in ["x", "y", "z"]], [getattr(msg.pose.pose.orientation, attr) for attr in ["x", "y", "z", "w"]])
                new_odom = copy.deepcopy(msg)
                new_odom.header.frame_id = self.odom_frame
                new_odom.child_frame_id = self.base_link_frame
                twist_list = [new_odom.twist.twist.linear.x, new_odom.twist.twist.linear.y, new_odom.twist.twist.linear.z, new_odom.twist.twist.angular.x, new_odom.twist.twist.angular.y, new_odom.twist.twist.angular.z]
                
                # use median filter to cancel spike noise of twist when use_twist_filter is true
                if self.use_twist_filter:                    
                    twist_list = self.median_filter(twist_list)
                    new_odom.twist.twist = Twist(Vector3(*twist_list[0:3]), Vector3(*twist_list[3: 6]))
                    
                # overwrite twist covariance when calculate_covariance flag is True
                if self.overwrite_pdf:
                    if not all([abs(x) < 1e-3 for x in twist_list]):
                        # shift twist according to error mean when moving (stopping state is trusted)
                        twist_list = [x + y for x, y in zip(twist_list, self.v_err_mean)]
                        new_odom.twist.twist = Twist(Vector3(*twist_list[0:3]), Vector3(*twist_list[3: 6]))
                    # calculate twist covariance according to standard diviation
                    if self.twist_proportional_sigma:
                        current_sigma = [x * y for x, y in zip(twist_list, self.v_err_sigma)]
                    else:
                        if all([abs(x) < 1e-3 for x in twist_list]):
                            current_sigma = [1e-6] * 6 # trust stopping state
                        else:
                            current_sigma = self.v_err_sigma
                    new_odom.twist.covariance = update_twist_covariance(new_odom.twist, current_sigma)
                    
                # offset coords
                new_odom_matrix = self.offset_matrix.dot(source_odom_matrix)
                new_odom.pose.pose.position = Point(*list(new_odom_matrix[:3, 3]))
                new_odom.pose.pose.orientation = Quaternion(*list(tf.transformations.quaternion_from_matrix(new_odom_matrix)))

                if self.overwrite_pdf:
                    if self.prev_odom != None:
                        dt = (new_odom.header.stamp - self.prev_odom.header.stamp).to_sec()
                        global_twist_with_covariance = TwistWithCovariance(transform_local_twist_to_global(new_odom.pose.pose, new_odom.twist.twist),
                                                                           transform_local_twist_covariance_to_global(new_odom.pose.pose, new_odom.twist.covariance))
                        new_odom.pose.covariance = update_pose_covariance(self.prev_odom.pose.covariance, global_twist_with_covariance.covariance, dt)
                    else:
                        new_odom.pose.covariance = numpy.diag([0.01**2] * 6).reshape(-1,).tolist() # initial covariance is assumed to be constant
                else:
                    # only offset pose covariance
                    new_pose_cov_matrix = numpy.matrix(new_odom.pose.covariance).reshape(6, 6)
                    rotation_matrix = self.offset_matrix[:3, :3]
                    new_pose_cov_matrix[:3, :3] = (rotation_matrix.T).dot(new_pose_cov_matrix[:3, :3].dot(rotation_matrix))
                    new_pose_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(new_pose_cov_matrix[3:6, 3:6].dot(rotation_matrix))
                    new_odom.pose.covariance = numpy.array(new_pose_cov_matrix).reshape(-1,).tolist()

                # publish
                self.pub.publish(new_odom)
                if self.publish_tf:
                    broadcast_transform(self.broadcast, new_odom, self.invert_tf)
                self.prev_odom = new_odom
                
            else:
                self.offset_matrix = self.calculate_offset(msg)

    def median_filter(self, data):
        self.filter_buffer.append(data)
        ret = numpy.median(self.filter_buffer, axis = 0)
        if len(self.filter_buffer) >= self.filter_buffer_size:
            self.filter_buffer.pop(0) # filter_buffer has at least 1 member
        return ret
