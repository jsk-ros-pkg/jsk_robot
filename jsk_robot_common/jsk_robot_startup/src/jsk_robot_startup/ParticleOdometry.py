#! /usr/bin/env python

import rospy
import numpy
import scipy.stats
import math
import random
import threading
import itertools
import tf
import time
import copy

from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Twist, Pose, Point, Quaternion, Vector3

# scipy.stats.multivariate_normal only can be used after SciPy 0.14.0
# input: x(array), mean(array), cov(matrix) output: probability of x
def norm_pdf_multivariate(x, mean, cov):
    size = len(x)
    if size == len(mean) and (size, size) == cov.shape:
        det = numpy.linalg.det(cov)
        if not det > 0:
            rospy.logwarn("Determinant {0} is equal or smaller than zero".format(det))
            return 0.0
        norm_const = math.pow((2*numpy.pi),float(size)/2) * math.pow(det,1.0/2)
        if not norm_const > 0 :
            rospy.logwarn("Norm const {0} is equal or smaller than zero".format(norm_const))
            return 0.0
        x_mean = numpy.matrix(x - mean)
        inv = cov.I
        exponent = -0.5 * (x_mean * inv * x_mean.T)
        if exponent > 0:
            rospy.logwarn("Exponent {0} is larger than zero".format(exponent))
            exponent = 0
        result = math.pow(math.e, exponent)
        return result / norm_const
    else:
        rospy.logwarn("The dimensions of the input don't match")
        return 0.0

class ParticleOdometry(object):
    ## initialize
    def __init__(self):
        # init node
        rospy.init_node("ParticleOdometry", anonymous=True)
        # instance valiables
        self.rate = float(rospy.get_param("~rate", 100))
        self.particle_num = float(rospy.get_param("~particle_num", 100))
        self.odom_frame = rospy.get_param("~odom_frame", "feedback_odom")
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.odom_init_frame = rospy.get_param("~odom_init_frame", "odom_init")
        self.z_error_sigma = rospy.get_param("~z_error_sigma", 0.01) # z error probability from source. z is assumed not to move largely from source odometry
        self.use_imu = rospy.get_param("~use_imu", False)
        self.roll_error_sigma = rospy.get_param("~roll_error_sigma", 0.05) # roll error probability from imu. (referenced only when use_imu is True)
        self.pitch_error_sigma = rospy.get_param("~pitch_error_sigma", 0.05) # pitch error probability from imu. (referenced only when use_imu is True)
        self.min_weight = rospy.get_param("~min_weight", 1e-10)
        self.r = rospy.Rate(self.rate)
        self.lock = threading.Lock()
        self.odom = None
        self.source_odom = None
        self.measure_odom = None
        self.particles = None
        self.weights = []
        self.measurement_updated = False
        self.init_sigma = [rospy.get_param("~init_sigma_x", 0.3),
                           rospy.get_param("~init_sigma_y", 0.3),
                           rospy.get_param("~init_sigma_z", 0.0001),
                           rospy.get_param("~init_sigma_roll", 0.0001),
                           rospy.get_param("~init_sigma_pitch", 0.0001),
                           rospy.get_param("~init_sigma_yaw", 0.2)]
        # tf
        self.listener = tf.TransformListener(True, rospy.Duration(10))
        self.broadcast = tf.TransformBroadcaster()
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.invert_tf = rospy.get_param("~invert_tf", True)
        # publisher
        self.pub = rospy.Publisher("~output", Odometry, queue_size=10)
        # subscriber
        self.source_odom_sub = rospy.Subscriber("~source_odom", Odometry, self.source_odom_callback, queue_size = 100)
        self.measure_odom_sub = rospy.Subscriber("~measure_odom", Odometry, self.measure_odom_callback, queue_size = 100)
        self.imu_sub = rospy.Subscriber("~imu", Imu, self.imu_callback, queue_size = 100)
        self.init_signal_sub = rospy.Subscriber("~init_signal", Empty, self.init_signal_callback, queue_size = 10)
        # init
        self.initialize_odometry()

    def initialize_odometry(self):
        try:
            (trans,rot) = self.listener.lookupTransform(self.odom_init_frame, self.base_link_frame, rospy.Time(0))
        except:
            rospy.logwarn("[%s] failed to solve tf in initialize_odometry: %s to %s", rospy.get_name(), self.odom_init_frame, self.base_link_frame)
            trans = [0.0, 0.0, 0.0]
            rot = [0.0, 0.0, 0.0, 1.0]
        rospy.loginfo("[%s]: initiailze odometry ", rospy.get_name())
        with self.lock:
            self.particles = self.initial_distribution(Pose(Point(*trans), Quaternion(*rot)))
            self.weights = [1.0 / self.particle_num] * int(self.particle_num)
            self.odom = Odometry()
            mean, cov = self.guess_normal_distribution(self.particles)
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
            
    ## particle filter functions
    # input: particles(list of pose), source_odom(control input)  output: list of sampled particles(pose)
    def sampling(self, particles, source_odom):
        current_time = source_odom.header.stamp
        global_twist_with_covariance = self.transform_twist_with_covariance_to_global(source_odom.pose, source_odom.twist)
        return [self.state_transition_probability_rvs(prt, global_twist_with_covariance.twist, global_twist_with_covariance.covariance, current_time) for prt in particles]
        
    # input: particles(list of pose), source_odom(control input), measure_odom(measurement),  min_weight(float) output: list of weights
    def weighting(self, particles, min_weight):
        if not self.measure_odom:
            rospy.logwarn("[%s] measurement does not exist.", rospy.get_name())
            weights = [1.0 / self.particle_num] * int(self.particle_num) # use uniform weights when measure_odom has not been subscribed yet
        else:
            measure_to_source_dt = (self.source_odom.header.stamp - self.measure_odom.header.stamp).to_sec() # adjust timestamp of pose in measure_odom to source_odom
            current_measure_pose_with_covariance = self.update_pose_with_covariance(self.measure_odom.pose, self.measure_odom.twist, measure_to_source_dt) # assuming dt is small and measure_odom.twist is do not change in dt
            weights = [max(min_weight, self.calculate_weighting_likelihood(prt, current_measure_pose_with_covariance.pose, current_measure_pose_with_covariance.covariance)) for prt in particles]
            if all([x == min_weight for x in weights]):
                rospy.logwarn("[%s] likelihood is too small and all weights are limited by min_weight.", rospy.get_name())
            normalization_coeffs = sum(weights) # normalization and each weight is assumed to be larger than 0
            weights = [w / normalization_coeffs for w in weights]
        return weights

    def calculate_weighting_likelihood(self, prt, measurement_pose, measurement_cov):
        measurement_likelihood = self.measurement_pdf(prt, measurement_pose, measurement_cov)
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
    # input: prev_x(pose), u(twist), u_cov(twist.covariance)  output: sampled x
    def state_transition_probability_rvs(self, prev_x, u, u_cov, current_time): # rvs = Random Varieties Sampling
        dt = (current_time - self.odom.header.stamp).to_sec()
        pose_list = self.convert_pose_to_list(prev_x)
        u_mean = [u.linear.x, u.linear.y, u.linear.z,
                  u.angular.x, u.angular.y, u.angular.z]
        u_cov_matrix = zip(*[iter(u_cov)]*6)
        vel_list = numpy.random.multivariate_normal(u_mean, u_cov_matrix, 1)
        ret_pose = [x + v * dt for x, v in zip(pose_list, vel_list[0])]
        return self.convert_list_to_pose(ret_pose)

    # input: x(pose), mean(pose), cov(pose.covariance), output: pdf value for x
    def measurement_pdf(self, x, measure_mean, measure_cov): # pdf = Probability Dencity Function
        # w ~ p(z(t)|x(t))
        x_array = numpy.array(self.convert_pose_to_list(x))
        mean_array = numpy.array(self.convert_pose_to_list(measure_mean))
        cov_matrix = numpy.matrix(measure_cov).reshape(len(x_array), len(x_array))
        pdf_value = norm_pdf_multivariate(x_array, mean_array, cov_matrix) # ~ p(x(t)|z(t))
        return pdf_value

    def z_error_pdf(self, particle_z):
        z_error = particle_z - self.source_odom.pose.pose.position.z
        return scipy.stats.norm.pdf(z_error, loc = 0.0, scale = self.z_error_sigma) # scale is standard divasion

    def imu_error_pdf(self, prt):
        if not self.imu:
            rospy.logwarn("[%s]: use_imu is True but imu is not subscribed", rospy.get_name())
            return 1.0 # multiply 1.0 make no effects to weight
        prt_euler = self.convert_pose_to_list(prt)[3:6]
        imu_euler = tf.transformations.euler_from_quaternion([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w]) # imu.orientation is assumed to be global
        return scipy.stats.norm.pdf(prt_euler[0] - imu_euler[0], loc = 0.0, scale = self.roll_error_sigma) * scipy.stats.norm.pdf(prt_euler[1] - imu_euler[1], loc = 0.0, scale = self.pitch_error_sigma)

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
        # estimate gaussian distribution for Odometry msg 
        mean, cov = self.guess_normal_distribution(self.particles)
        self.odom.pose.pose = self.convert_list_to_pose(mean)
        self.odom.pose.covariance = list(itertools.chain(*cov))
        self.pub.publish(self.odom)
        if self.publish_tf:
            self.broadcast_transform()

    ## callback functions
    def source_odom_callback(self, msg):
        with self.lock:
            self.source_odom = msg

    def measure_odom_callback(self, msg):
        with self.lock:
            self.measure_odom = msg
            self.measurement_updated = True # raise measurement flag

    def init_signal_callback(self, msg):
        # time.sleep(1) # wait to update odom_init frame
        self.initialize_odometry()

    def imu_callback(self, msg):
        with self.lock:
            self.imu = msg
        
    # main functions
    def update(self):
        if not self.odom or not self.particles or not self.source_odom:
            rospy.logwarn("[%s]: odometry is not initialized", rospy.get_name())
            return
        else:
            self.calc_odometry()
            self.publish_odometry()

    def execute(self):
        while not rospy.is_shutdown():
            with self.lock:
                self.update() # call update() when control input is subscribed
            self.r.sleep()
        
    ## utils
    def convert_pose_to_list(self, pose):
        euler = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y,
                                                          pose.orientation.z, pose.orientation.w))
        return [pose.position.x, pose.position.y, pose.position.z, euler[0], euler[1], euler[2]]

    def convert_list_to_pose(self, lst):
        return Pose(Point(*lst[0:3]), Quaternion(*tf.transformations.quaternion_from_euler(*lst[3:6])))

    def guess_normal_distribution(self, particles):
        particles_lst = [self.convert_pose_to_list(prt) for prt in particles]
        mean = numpy.mean(particles_lst, axis = 0)
        cov = numpy.cov(particles_lst, rowvar = 0)
        return (mean, cov)

    def broadcast_transform(self):
        if not self.odom:
            return
        position = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z]
        orientation = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
        if self.invert_tf:
            homogeneous_matrix = tf.transformations.quaternion_matrix(orientation)
            homogeneous_matrix[:3, 3] = numpy.array(position).reshape(1, 3)
            homogeneous_matrix_inv = numpy.linalg.inv(homogeneous_matrix)
            position = list(homogeneous_matrix_inv[:3, 3])
            orientation = list(tf.transformations.quaternion_from_matrix(homogeneous_matrix_inv))
            parent_frame = self.odom.child_frame_id
            target_frame = self.odom.header.frame_id
        else:
            parent_frame = self.odom.header.frame_id
            target_frame = self.odom.child_frame_id
        self.broadcast.sendTransform(position, orientation, rospy.Time.now(), target_frame, parent_frame)

    def transform_twist_with_covariance_to_global(self, pose, twist):
        trans = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        rot = [pose.pose.orientation.x, pose.pose.orientation.y,
               pose.pose.orientation.z, pose.pose.orientation.w]
        rotation_matrix = tf.transformations.quaternion_matrix(rot)[:3, :3]
        twist_cov_matrix = numpy.matrix(twist.covariance).reshape(6, 6)
        global_velocity = numpy.dot(rotation_matrix, numpy.array([[twist.twist.linear.x],
                                                                  [twist.twist.linear.y],
                                                                  [twist.twist.linear.z]]))
        global_omega = numpy.dot(rotation_matrix, numpy.array([[twist.twist.angular.x],
                                                               [twist.twist.angular.y],
                                                               [twist.twist.angular.z]]))
        global_twist_cov_matrix = numpy.zeros((6, 6))
        global_twist_cov_matrix[:3, :3] = (rotation_matrix.T).dot(twist_cov_matrix[:3, :3].dot(rotation_matrix))
        global_twist_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(twist_cov_matrix[3:6, 3:6].dot(rotation_matrix))

        return TwistWithCovariance(Twist(Vector3(*global_velocity[:, 0]), Vector3(*global_omega[:, 0])),
                                   global_twist_cov_matrix.reshape(-1,).tolist())

    def update_pose_with_covariance(self, pose_with_covariance, twist_with_covariance, dt):
        ret_pose = Pose()
        global_twist_with_covariance = self.transform_twist_with_covariance_to_global(pose_with_covariance, twist_with_covariance)
        euler = list(tf.transformations.euler_from_quaternion((pose_with_covariance.pose.orientation.x, pose_with_covariance.pose.orientation.y,
                                                               pose_with_covariance.pose.orientation.z, pose_with_covariance.pose.orientation.w)))
        # calculate current pose as integration
        ret_pose.position.x = pose_with_covariance.pose.position.x + global_twist_with_covariance.twist.linear.x * dt
        ret_pose.position.y = pose_with_covariance.pose.position.y + global_twist_with_covariance.twist.linear.y * dt
        ret_pose.position.z = pose_with_covariance.pose.position.z + global_twist_with_covariance.twist.linear.z * dt
        euler[0] += global_twist_with_covariance.twist.angular.x * dt
        euler[1] += global_twist_with_covariance.twist.angular.y * dt
        euler[2] += global_twist_with_covariance.twist.angular.z * dt
        quat = tf.transformations.quaternion_from_euler(*euler)
        ret_pose.orientation = Quaternion(*quat)

        # update covariance
        ret_pose_cov = []
        # make matirx from covarinace array
        prev_pose_cov_matrix = numpy.matrix(pose_with_covariance.covariance).reshape(6, 6)
        global_twist_cov_matrix = numpy.matrix(global_twist_with_covariance.covariance).reshape(6, 6)
        # jacobian matrix
        # elements in pose and twist are assumed to be independent on global coordinates
        jacobi_pose = numpy.diag([1.0] * 6)
        jacobi_twist = numpy.diag([dt] * 6)
        # covariance calculation
        pose_cov_matrix = jacobi_pose.dot(prev_pose_cov_matrix.dot(jacobi_pose.T)) + jacobi_twist.dot(global_twist_cov_matrix.dot(jacobi_twist.T))
        # update covariances as array type (twist is same as before)
        ret_pose_cov = numpy.array(pose_cov_matrix).reshape(-1,).tolist()

        return PoseWithCovariance(ret_pose, ret_pose_cov)

