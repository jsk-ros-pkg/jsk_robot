#! /usr/bin/env python

import rospy
import numpy
import scipy.stats
import random
import threading
import itertools
import tf
import time
import copy
from operator import itemgetter 

from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Twist, Pose, Point, Quaternion, Vector3, TransformStamped
from jsk_recognition_msgs.msg import HistogramWithRangeArray, HistogramWithRange, HistogramWithRangeBin
from odometry_utils import norm_pdf_multivariate, transform_quaternion_to_euler, transform_local_twist_to_global, transform_local_twist_covariance_to_global, update_pose, update_pose_covariance, broadcast_transform

class ParticleOdometry(object):
    ## initialize
    def __init__(self):
        # init node
        rospy.init_node("ParticleOdometry", anonymous=True)
        # instance valiables
        self.rate = float(rospy.get_param("~rate", 100))
        self.particle_num = float(rospy.get_param("~particle_num", 100))
        self.valid_particle_num = min(float(rospy.get_param("~valid_particle_num", int(self.particle_num / 2.0))), self.particle_num) # select valid_particle_num particles in ascending weight sequence when estimating normal distributions
        self.odom_frame = rospy.get_param("~odom_frame", "feedback_odom")
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.odom_init_frame = rospy.get_param("~odom_init_frame", "odom_init")
        self.z_error_sigma = rospy.get_param("~z_error_sigma", 0.01) # z error probability from source. z is assumed not to move largely from source odometry
        self.use_imu = rospy.get_param("~use_imu", False)
        self.use_imu_yaw = rospy.get_param("~use_imu_yaw", False) # referenced only when use_imu is True
        self.roll_error_sigma = rospy.get_param("~roll_error_sigma", 0.05) # roll error probability from imu. (referenced only when use_imu is True)
        self.pitch_error_sigma = rospy.get_param("~pitch_error_sigma", 0.05) # pitch error probability from imu. (referenced only when use_imu is True)
        self.yaw_error_sigma = rospy.get_param("~yaw_error_sigma", 0.1) # yaw error probability from imu. (referenced only when use_imu and use_imu_yaw are both True)
        self.min_weight = rospy.get_param("~min_weight", 1e-10)
        self.r = rospy.Rate(self.rate)
        self.lock = threading.Lock()
        self.odom = None
        self.source_odom = None
        self.measure_odom = None
        self.imu = None
        self.imu_rotation = None
        self.particles = None
        self.weights = []
        self.measurement_updated = False
        self.prev_rpy = None
        self.init_sigma = [rospy.get_param("~init_sigma_x", 0.1),
                           rospy.get_param("~init_sigma_y", 0.1),
                           rospy.get_param("~init_sigma_z", 0.0001),
                           rospy.get_param("~init_sigma_roll", 0.0001),
                           rospy.get_param("~init_sigma_pitch", 0.0001),
                           rospy.get_param("~init_sigma_yaw", 0.05)]
        # tf
        self.publish_tf = rospy.get_param("~publish_tf", True)
        if self.publish_tf:
            self.broadcast = tf.TransformBroadcaster()
            self.invert_tf = rospy.get_param("~invert_tf", True)
        # publisher
        self.pub = rospy.Publisher("~output", Odometry, queue_size = 1)
        # histogram
        self.publish_histogram = rospy.get_param("~publish_histogram", False)
        if self.publish_histogram:
            self.pub_hist = rospy.Publisher("~particle_histograms", HistogramWithRangeArray, queue_size = 1)
        # subscriber
        self.source_odom_sub = rospy.Subscriber("~source_odom", Odometry, self.source_odom_callback, queue_size = 10)
        self.measure_odom_sub = rospy.Subscriber("~measure_odom", Odometry, self.measure_odom_callback, queue_size = 10)
        self.imu_sub = rospy.Subscriber("~imu", Imu, self.imu_callback, queue_size = 10) # imu is assumed to be in base_link_frame relative coords
        self.init_transform_sub = rospy.Subscriber("~initial_base_link_transform", TransformStamped, self.init_transform_callback) # init_transform is assumed to be transform of init_odom -> base_link
        # init
        self.initialize_odometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])

    def initialize_odometry(self, trans, rot):
        with self.lock:
            self.particles = self.initial_distribution(Pose(Point(*trans), Quaternion(*rot)))
            self.weights = [1.0 / self.particle_num] * int(self.particle_num)
            self.odom = Odometry()
            mean, cov = self.guess_normal_distribution(self.particles, self.weights)
            self.odom.pose.pose = self.convert_list_to_pose(mean)
            self.odom.pose.covariance = list(itertools.chain(*cov))
            self.odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            self.odom.twist.covariance = numpy.diag([0.001**2]*6).reshape(-1,).tolist()
            self.odom.header.stamp = rospy.Time.now()
            self.odom.header.frame_id = self.odom_frame
            self.odom.child_frame_id = self.base_link_frame
            self.source_odom = None
            self.measure_odom = None
            self.measurement_updated = False
            self.imu = None
            self.imu_rotation = None
            self.prev_rpy = None
            
    ## particle filter functions
    # input: particles(list of pose), source_odom(control input)  output: list of sampled particles(pose)
    def sampling(self, particles, source_odom):
        global_twist_with_covariance = self.transform_twist_with_covariance_to_global(source_odom.pose, source_odom.twist)
        sampled_velocities = self.state_transition_probability_rvs(global_twist_with_covariance.twist, global_twist_with_covariance.covariance) # make sampeld velocity at once because multivariate_normal calculates invert matrix and it is slow
        dt = (source_odom.header.stamp - self.odom.header.stamp).to_sec()
        return [update_pose(prt, Twist(Vector3(*vel[0:3]), Vector3(*vel[3:6])), dt) for prt, vel in zip(particles, sampled_velocities)]
        
    # input: particles(list of pose), min_weight(float) output: list of weights
    def weighting(self, particles, min_weight):
        if not self.measure_odom:
            rospy.logwarn("[%s] measurement does not exist.", rospy.get_name())
            weights = [1.0 / self.particle_num] * int(self.particle_num) # use uniform weights when measure_odom has not been subscribed yet
        else:
            measure_to_source_dt = (self.source_odom.header.stamp - self.measure_odom.header.stamp).to_sec() # adjust timestamp of pose in measure_odom to source_odom
            current_measure_pose_with_covariance = self.update_pose_with_covariance(self.measure_odom.pose, self.measure_odom.twist, measure_to_source_dt) # assuming dt is small and measure_odom.twist is do not change in dt
            measurement_pose_array = numpy.array(self.convert_pose_to_list(current_measure_pose_with_covariance.pose))
            try:
                measurement_cov_matrix_inv = numpy.linalg.inv(numpy.matrix(current_measure_pose_with_covariance.covariance).reshape(6, 6)) # calculate inverse matrix first to reduce computation cost
                weights = [max(min_weight, self.calculate_weighting_likelihood(prt, measurement_pose_array, measurement_cov_matrix_inv)) for prt in particles]
            except numpy.linalg.LinAlgError:
                rospy.logwarn("[%s] covariance matrix is not singular.", rospy.get_name())
                weights = [min_weight] * len(particles)
            if all([x == min_weight for x in weights]):
                rospy.logwarn("[%s] likelihood is too small and all weights are limited by min_weight.", rospy.get_name())
            normalization_coeffs = sum(weights) # normalization and each weight is assumed to be larger than 0
            weights = [w / normalization_coeffs for w in weights]
        return weights

    def calculate_weighting_likelihood(self, prt, measurement_pose_array, measurement_cov_matrix_inv):
        measurement_likelihood = self.measurement_pdf(prt, measurement_pose_array, measurement_cov_matrix_inv)
        z_error_likelihood = self.z_error_pdf(prt.position.z) # consider difference from ideal z height to prevent drift
        if self.use_imu:
            imu_likelihood = self.imu_error_pdf(prt)
            return measurement_likelihood * z_error_likelihood * imu_likelihood
        else:
            return  measurement_likelihood * z_error_likelihood

    # input: list of particles, list of weights output: list of particles
    def resampling(self, particles, weights):
        uniform_probability = 1.0 / self.particle_num
        ret_particles = []
        probability_seed = numpy.random.rand() * uniform_probability
        weight_amount = self.weights[0]
        index = 0
        # for i in range(int(self.particle_num)):
        for i in range(int(self.particle_num)):
            selector = probability_seed + i * uniform_probability
            while selector > weight_amount and index < len(weights):
                index += 1
                weight_amount += weights[index]
            ret_particles.append(particles[index])
        return ret_particles

    ## probability functions
    # input: u(twist), u_cov(twist.covariance)  output: sampled velocity
    def state_transition_probability_rvs(self, u, u_cov): # rvs = Random Varieties Sampling
        u_mean = [u.linear.x, u.linear.y, u.linear.z,
                  u.angular.x, u.angular.y, u.angular.z]
        u_cov_matrix = zip(*[iter(u_cov)]*6)
        return numpy.random.multivariate_normal(u_mean, u_cov_matrix, int(self.particle_num)).tolist()

    # input: x(pose), mean(array), cov_inv(matrix), output: pdf value for x
    def measurement_pdf(self, x, measure_mean_array, measure_cov_matrix_inv): # pdf = Probability Dencity Function
        # w ~ p(z(t)|x(t))
        x_array = numpy.array(self.convert_pose_to_list(x))
        pdf_value = norm_pdf_multivariate(x_array, measure_mean_array, measure_cov_matrix_inv) # ~ p(x(t)|z(t))
        return pdf_value

    def z_error_pdf(self, particle_z):
        z_error = particle_z - self.source_odom.pose.pose.position.z
        return scipy.stats.norm.pdf(z_error, loc = 0.0, scale = self.z_error_sigma) # scale is standard divasion

    def imu_error_pdf(self, prt):
        if not self.imu:
            rospy.logwarn("[%s]: use_imu is True but imu is not subscribed", rospy.get_name())
            return 1.0 # multiply 1.0 make no effects to weight
        prt_euler = self.convert_pose_to_list(prt)[3:6]
        imu_matrix = tf.transformations.quaternion_matrix([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w])[:3, :3]
        imu_euler = tf.transformations.euler_from_matrix(numpy.dot(self.imu_rotation, imu_matrix)) # imu is assumed to be in base_link relative and imu_rotation is base_link->particle_odom transformation
        roll_pitch_pdf = scipy.stats.norm.pdf(prt_euler[0] - imu_euler[0], loc = 0.0, scale = self.roll_error_sigma) * scipy.stats.norm.pdf(prt_euler[1] - imu_euler[1], loc = 0.0, scale = self.pitch_error_sigma)
        if self.use_imu_yaw:
            return roll_pitch_pdf * scipy.stats.norm.pdf(prt_euler[2] - imu_euler[2], loc = 0.0, scale = self.yaw_error_sigma)
        else:
            return roll_pitch_pdf

    # input: init_pose(pose), output: initial distribution of pose(list of pose)
    def initial_distribution(self, init_pose):
        pose_list = numpy.random.multivariate_normal(numpy.array(self.convert_pose_to_list(init_pose)), numpy.diag([x ** 2 for x in self.init_sigma]), int(self.particle_num))
        return [self.convert_list_to_pose(pose) for pose in pose_list]

    ## top odometry calculations 
    def calc_odometry(self):
        # sampling
        self.particles = self.sampling(self.particles, self.source_odom)
        if self.measurement_updated:
            # weighting
            self.weights = self.weighting(self.particles, self.min_weight)
            # resampling
            self.particles = self.resampling(self.particles, self.weights)
            # wait next measurement
            self.measurement_updated = False

    def publish_odometry(self):
        # relfect source_odom information
        self.odom.header.stamp = self.source_odom.header.stamp
        self.odom.twist = self.source_odom.twist
        # use only important particels
        combined_prt_weight = zip(self.particles, self.weights)
        selected_prt_weight = zip(*sorted(combined_prt_weight, key = itemgetter(1), reverse = True)[:int(self.valid_particle_num)]) # [(p0, w0), (p1, w1), ..., (pN, wN)] -> [(sorted_p0, sorted_w0), (sorted_p1, sorted_w1), ..., (sorted_pN', sorted_wN')] -> [(sorted_p0, ..., sorted_pN'), (sorted_w0, ..., sorted_wN')]
        # estimate gaussian distribution for Odometry msg 
        mean, cov = self.guess_normal_distribution(selected_prt_weight[0], selected_prt_weight[1])
        self.odom.pose.pose = self.convert_list_to_pose(mean)
        self.odom.pose.covariance = list(itertools.chain(*cov))
        self.pub.publish(self.odom)
        if self.publish_tf:
            broadcast_transform(self.broadcast, self.odom, self.invert_tf)
        # update prev_rpy to prevent jump of angles
        self.prev_rpy = transform_quaternion_to_euler([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w], self.prev_rpy)

    ## callback functions
    def source_odom_callback(self, msg):        
        with self.lock:
            self.source_odom = msg

    def measure_odom_callback(self, msg):
        with self.lock:
            self.measure_odom = msg
            self.measurement_updated = True # raise measurement flag

    def init_transform_callback(self, msg):
        self.initialize_odometry([getattr(msg.transform.translation, attr) for attr in ["x", "y", "z"]],
                                 [getattr(msg.transform.rotation, attr) for attr in ["x", "y", "z", "w"]])

    def imu_callback(self, msg):
        with self.lock:
            if self.odom == None:
                return # cannot calculate imu_rotation
            self.imu_rotation = numpy.linalg.inv(tf.transformations.quaternion_matrix([getattr(self.odom.pose.pose.orientation, attr) for attr in ["x", "y", "z", "w"]])[:3, :3]) # base_link -> particle_odom
            self.imu = msg
        
    # main functions
    def update(self):
        if not self.odom or not self.particles or not self.source_odom:
            rospy.logwarn("[%s]: odometry is not initialized", rospy.get_name())
            return
        else:
            self.calc_odometry()
            self.publish_odometry()
            if self.publish_histogram:
                histgram_msg = self.make_histogram_array(self.particles, self.source_odom.header.stamp)
                self.pub_hist.publish(histgram_msg)

    def execute(self):
        while not rospy.is_shutdown():
            with self.lock:
                self.update() # call update() when control input is subscribed
            self.r.sleep()
        
    ## utils
    def convert_pose_to_list(self, pose):
        euler = transform_quaternion_to_euler((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), self.prev_rpy)
        return [pose.position.x, pose.position.y, pose.position.z, euler[0], euler[1], euler[2]]

    def convert_list_to_pose(self, lst):
        return Pose(Point(*lst[0:3]), Quaternion(*tf.transformations.quaternion_from_euler(*lst[3:6])))

    def guess_normal_distribution(self, particles, weights):
        # particles_lst = [self.convert_pose_to_list(prt) for prt in particles]
        # mean = numpy.mean(particles_lst, axis = 0)
        # cov = numpy.cov(particles_lst, rowvar = 0)
        
        # particles_list = [numpy.array(self.convert_pose_to_list(prt)) for prt in particles]
        # mean = None
        # cov = None
        # w2_sum = 0.0
        # mean = numpy.average(particles_list, axis = 0, weights = weights)
        # for prt, w in zip(particles_list, weights):
        #     if cov == None:
        #         cov = w * numpy.vstack(prt - mean) * (prt - mean)
        #     else:
        #         cov += w * numpy.vstack(prt - mean) * (prt - mean)
        #     w2_sum += w ** 2
        # cov = (1.0 / (1.0 - w2_sum)) * cov # unbiased covariance

        # calculate weighted mean and covariance (cf. https://en.wikipedia.org/wiki/Weighted_arithmetic_mean#Weighted_sample_covariance)        
        particle_array = numpy.array([self.convert_pose_to_list(prt) for prt in particles])
        weights_array = numpy.array(weights)
        mean = numpy.average(particle_array, axis = 0, weights = weights_array)
        diffs = particle_array - mean # array of x - mean
        cov = numpy.dot((numpy.vstack(weights_array) * diffs).T, diffs) # sum(w * (x - mean).T * (x - mean))
        cov = (1.0 / (1.0 - sum([w ** 2 for w in weights]))) * cov # unbiased covariance
        
        return (mean.tolist(), cov.tolist())

    def transform_twist_with_covariance_to_global(self, pose_with_covariance, twist_with_covariance):
        global_twist = transform_local_twist_to_global(pose_with_covariance.pose, twist_with_covariance.twist)
        global_twist_cov = transform_local_twist_covariance_to_global(pose_with_covariance.pose, twist_with_covariance.covariance)
        return TwistWithCovariance(global_twist, global_twist_cov)

    def update_pose_with_covariance(self, pose_with_covariance, twist_with_covariance, dt):
        global_twist_with_covariance = self.transform_twist_with_covariance_to_global(pose_with_covariance, twist_with_covariance)
        ret_pose = update_pose(pose_with_covariance.pose, global_twist_with_covariance.twist, dt)
        ret_pose_cov = update_pose_covariance(pose_with_covariance.covariance, global_twist_with_covariance.covariance, dt)
        return PoseWithCovariance(ret_pose, ret_pose_cov)
    def make_histogram_array(self, particles, stamp):
        # initialize
        histogram_array_msg = HistogramWithRangeArray()
        histogram_array_msg.header.frame_id = self.odom_frame
        histogram_array_msg.header.stamp = stamp
        for i in range(6):
            range_msg = HistogramWithRange()
            range_msg.header.frame_id = self.odom_frame
            range_msg.header.stamp = stamp
            histogram_array_msg.histograms.append(range_msg)
        # count
        particle_array = numpy.array([self.convert_pose_to_list(prt) for prt in particles])
        data = zip(*particle_array) # [(x_data), (y_data), ..., (yaw_data)]
        for i, d in enumerate(data):
            hist, bins = numpy.histogram(d, bins=50)
            for count, min_value, max_value in zip(hist, bins[:-1], bins[1:]):
                msg_bin = HistogramWithRangeBin()
                msg_bin.max_value = max_value
                msg_bin.min_value = min_value
                msg_bin.count = count
                histogram_array_msg.histograms[i].bins.append(msg_bin)
        return histogram_array_msg
