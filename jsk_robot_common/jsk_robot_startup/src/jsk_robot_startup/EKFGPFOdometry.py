#! /usr/bin/env python

import rospy
import numpy
import scipy.stats
import random
import threading
import itertools
import tf
import time
from operator import itemgetter 

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Twist, Pose, Point, Quaternion, Vector3, TransformStamped
from ParticleOdometry import ParticleOdometry
from odometry_utils import norm_pdf_multivariate, transform_quaternion_to_euler, transform_local_twist_to_global, transform_local_twist_covariance_to_global, update_pose, update_pose_covariance, broadcast_transform

class EKFGPFOdometry(ParticleOdometry):
    def __init__(self):
        ParticleOdometry.__init__(self)
        self.min_sampling_rate = float(rospy.get_param("~min_sampling_rate", self.rate)) # execute sampling every time topic is subscribed when this value is not larger than 0
        self.last_sampling_time = rospy.Time.now()

    def initialize_odometry(self, trans, rot):
        with self.lock:
            self.particles = None
            self.weights = []
            self.odom = Odometry()
            self.odom.pose.pose = Pose(Point(*trans), Quaternion(*rot))
            self.odom.pose.covariance = numpy.diag([x ** 2 for x in self.init_sigma]).reshape(-1,).tolist()
            self.odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            self.odom.twist.covariance = numpy.diag([0.001**2]*6).reshape(-1,).tolist()
            self.odom.header.stamp = rospy.Time.now()
            self.odom.header.frame_id = self.odom_frame
            self.odom.child_frame_id = self.base_link_frame
            self.source_odom = None
            self.measure_odom = None
            self.imu = None
            self.imu_rotation = None
            self.prev_rpy = None

    # EKF
    def source_odom_callback(self, msg):        
        with self.lock:
            self.source_odom = msg
            self.ekf_update(self.odom, self.source_odom)
    
    def ekf_update(self, current_odom, source_odom):
        dt = (source_odom.header.stamp - current_odom.header.stamp).to_sec()
        if dt > 0:
            # update current_odom (only update pose, stamp and twist are copied from source_odom)
            current_odom.header.stamp = source_odom.header.stamp
            current_odom.twist = source_odom.twist
            new_pose_with_covariance = self.update_pose_with_covariance(current_odom.pose, source_odom.twist, dt)
            current_odom.pose = new_pose_with_covariance
                    
    ## particle filter functions
    # sampling poses from EKF result (current_pose_with_covariance)
    def measure_odom_callback(self, msg):
        with self.lock:
            self.measure_odom = msg
            if not self.odom or not self.source_odom:
                return
            elif self.min_sampling_rate <= 0 or (self.measure_odom.header.stamp - self.last_sampling_time).to_sec() > 1.0 / self.min_sampling_rate:
                # sampling
                self.particles = self.sampling(self.odom.pose)
                # weighting
                self.weights = self.weighting(self.particles, self.min_weight)
                # resampling
                self.particles = self.resampling(self.particles, self.weights)
                # estimate new pdf 
                self.approximate_odometry(self.particles, self.weights)
                # wait next measurement
                self.last_sampling_time = self.measure_odom.header.stamp
            
    def sampling(self, current_pose_with_covariance):
        pose_mean = self.convert_pose_to_list(current_pose_with_covariance.pose)
        pose_cov_matrix = zip(*[iter(current_pose_with_covariance.covariance)]*6)
        return [self.convert_list_to_pose(x) for x in numpy.random.multivariate_normal(pose_mean, pose_cov_matrix, int(self.particle_num)).tolist()]

    def approximate_odometry(self, particles, weights):
        # use only important particels
        combined_prt_weight = zip(self.particles, self.weights)
        selected_prt_weight = zip(*sorted(combined_prt_weight, key = itemgetter(1), reverse = True)[:int(self.valid_particle_num)]) # [(p0, w0), (p1, w1), ..., (pN, wN)] -> [(sorted_p0, sorted_w0), (sorted_p1, sorted_w1), ..., (sorted_pN', sorted_wN')] -> [(sorted_p0, ..., sorted_pN'), (sorted_w0, ..., sorted_wN')]
        # estimate gaussian distribution for Odometry msg 
        mean, cov = self.guess_normal_distribution(selected_prt_weight[0], selected_prt_weight[1])
        # overwrite pose pdf
        self.odom.pose.pose = self.convert_list_to_pose(mean)
        self.odom.pose.covariance = list(itertools.chain(*cov))

    def publish_odometry(self):
        # refrect source_odom informations
        self.pub.publish(self.odom)
        if self.publish_tf:
            broadcast_transform(self.broadcast, self.odom, self.invert_tf)
        # diagnostics
        self.update_diagnostics(self.particles, self.weights, self.odom.header.stamp)
        # update prev_rpy to prevent jump of angles
        self.prev_rpy = transform_quaternion_to_euler([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w], self.prev_rpy)
        
    # main functions
    def update(self):
        if not self.odom or not self.source_odom:
            rospy.logwarn("[%s]: odometry is not initialized", rospy.get_name())
            return
        else:
            self.publish_odometry()
            if self.publish_histogram:
                histgram_msg = self.make_histogram_array(self.particles, self.source_odom.header.stamp)
                self.pub_hist.publish(histgram_msg)

    def execute(self):
        while not rospy.is_shutdown():
            with self.lock:
                self.update() # call update() when control input is subscribed
            self.r.sleep()
