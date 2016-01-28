#! /usr/bin/env python

# This script only calculate offset caused by odometry feedback and do not consider initial offset.
# Initial offset should be calculated by OdometryOffset.py (source_odom is assumed to be offseted already) 

import rospy
import numpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
import tf
import sys
import threading
import copy
from scipy import signal
from dynamic_reconfigure.server import Server
from jsk_robot_startup.cfg import OdometryFeedbackWrapperReconfigureConfig

from odometry_utils import transform_local_twist_to_global, transform_local_twist_covariance_to_global, update_pose, update_pose_covariance, broadcast_transform, make_homogeneous_matrix

class OdometryFeedbackWrapper(object):
    def __init__(self):
        rospy.init_node("OdometryFeedbackWrapper", anonymous=True)
        self.rate = float(rospy.get_param("~rate", 100))
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.odom_frame = rospy.get_param("~odom_frame", "feedback_odom")
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.max_feedback_time = rospy.get_param("~max_feedback_time", 60) # if max_feedback_time <= 0, feedback is not occurs by time
        self.tf_cache_time = rospy.get_param("~tf_cache_time", 60) # determined from frequency of feedback_odom
        if self.publish_tf:
            self.broadcast = tf.TransformBroadcaster()
            self.invert_tf = rospy.get_param("~invert_tf", True)
        self.odom = None # belief of this wrapper
        self.feedback_odom = None
        self.source_odom = None
        self.offset_homogeneous_matrix = None
        self.dt = 0.0
        self.prev_feedback_time = rospy.Time.now()
        self.r = rospy.Rate(self.rate)
        self.lock = threading.Lock()
        self.odom_history = []
        self.force_feedback_sigma = rospy.get_param("~force_feedback_sigma", 0.5)
        self.distribution_feedback_minimum_sigma = rospy.get_param("~distribution_feedback_minimum_sigma", 0.5)
        self.pub = rospy.Publisher("~output", Odometry, queue_size = 1)
        self.initialize_odometry()
        self.init_signal_sub = rospy.Subscriber("~init_signal", Empty, self.init_signal_callback, queue_size = 10)
        self.source_odom_sub = rospy.Subscriber("~source_odom", Odometry, self.source_odom_callback, queue_size = 10)
        self.feedback_odom_sub = rospy.Subscriber("~feedback_odom", Odometry, self.feedback_odom_callback, queue_size = 10)
        self.reconfigure_server = Server(OdometryFeedbackWrapperReconfigureConfig, self.reconfigure_callback)

    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def reconfigure_callback(self, config, level):
        self.force_feedback_sigma = config["force_feedback_sigma"]
        rospy.loginfo("[%s]: force feedback sigma is %f", rospy.get_name(), self.force_feedback_sigma)
        self.distribution_feedback_minimum_sigma = config["distribution_feedback_minimum_sigma"]
        rospy.loginfo("[%s]: distribution feedback minimum sigma is %f", rospy.get_name(), self.distribution_feedback_minimum_sigma)
        return config

    def init_signal_callback(self, msg):
        self.initialize_odometry()

    def initialize_odometry(self):
        with self.lock:
            self.odom_history = []
            self.prev_feedback_time = rospy.Time.now()
            self.offset_homogeneous_matrix = tf.transformations.quaternion_matrix([0, 0, 0, 1])
            self.odom = None
            self.source_odom = None
            self.feedback_odom = None

    def source_odom_callback(self, msg):
        with self.lock:
            self.source_odom = msg
            self.calculate_odometry(self.odom, self.source_odom)
            self.publish_odometry()
            if self.publish_tf:
                broadcast_transform(self.broadcast, self.odom, self.invert_tf)

    def feedback_odom_callback(self, msg):
        if not self.odom:
            return
        self.feedback_odom = msg
        with self.lock:
            # check distribution accuracy
            nearest_odom = copy.deepcopy(self.odom)
            nearest_dt = (self.feedback_odom.header.stamp - self.odom.header.stamp).to_sec()
            for hist in self.odom_history: # get neaerest odom from feedback_odom referencing timestamp
                dt = (self.feedback_odom.header.stamp - hist.header.stamp).to_sec()
                if abs(dt) < abs(nearest_dt):
                    nearest_dt = dt
                    nearest_odom = copy.deepcopy(hist)
            # get approximate pose at feedback_odom timestamp (probably it is past) of nearest_odom
            global_nearest_odom_twist = transform_local_twist_to_global(nearest_odom.pose.pose, nearest_odom.twist.twist)
            nearest_odom.pose.pose = update_pose(nearest_odom.pose.pose, global_nearest_odom_twist, nearest_dt)
            global_nearest_odom_twist_covariance = transform_local_twist_covariance_to_global(nearest_odom.pose.pose, nearest_odom.twist.covariance)
            nearest_odom.pose.covariance = update_pose_covariance(nearest_odom.pose.covariance, global_nearest_odom_twist_covariance, nearest_dt)
            enable_feedback = self.check_covariance(nearest_odom) or self.check_distribution_difference(nearest_odom, self.feedback_odom) or self.check_feedback_time()
            # update feedback_odom to approximate odom at current timestamp using previsous velocities
            if enable_feedback:
                rospy.loginfo("%s: Feedback enabled.", rospy.get_name())
                # adjust timestamp of self.feedback_odom to current self.odom
                for hist in self.odom_history:
                    dt = (hist.header.stamp - self.feedback_odom.header.stamp).to_sec()
                    if dt > 0.0:
                        # update pose and twist according to the history
                        self.update_twist(self.feedback_odom.twist, hist.twist)
                        global_hist_twist = transform_local_twist_to_global(hist.pose.pose, hist.twist.twist)
                        self.feedback_odom.pose.pose = update_pose(self.feedback_odom.pose.pose, global_hist_twist, dt) # update feedback_odom according to twist of hist
                        # update covariance
                        self.feedback_odom.twist.covariance = hist.twist.covariance
                        global_hist_twist_covariance = transform_local_twist_covariance_to_global(self.feedback_odom.pose.pose, hist.twist.covariance)
                        self.feedback_odom.pose.covariance = update_pose_covariance(self.feedback_odom.pose.covariance, global_hist_twist_covariance, dt)
                        self.feedback_odom.header.stamp = hist.header.stamp
                dt = (self.odom.header.stamp - self.feedback_odom.header.stamp).to_sec()
                global_feedback_odom_twist = transform_local_twist_to_global(self.feedback_odom.pose.pose, self.feedback_odom.twist.twist)
                self.feedback_odom.pose.pose = update_pose(self.feedback_odom.pose.pose, global_feedback_odom_twist, dt)
                global_feedback_odom_twist_covariance = transform_local_twist_covariance_to_global(self.feedback_odom.pose.pose, self.feedback_odom.twist.covariance)
                self.feedback_odom.pose.covariance = update_pose_covariance(self.feedback_odom.pose.covariance, global_feedback_odom_twist_covariance, dt)
                self.feedback_odom.header.stamp = self.odom.header.stamp
                # integrate feedback_odom and current odom
                new_odom_pose, new_odom_cov = self.calculate_mean_and_covariance(self.odom.pose, self.feedback_odom.pose)
                # update self.odom according to the result of integration
                quat = tf.transformations.quaternion_from_euler(*new_odom_pose[3:6])
                self.odom.pose.pose = Pose(Point(*new_odom_pose[0:3]), Quaternion(*quat))
                self.odom.pose.covariance = new_odom_cov
                self.prev_feedback_time = self.odom.header.stamp
                self.odom_history = []
                # update offset
                new_pose_homogeneous_matrix = make_homogeneous_matrix([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z],
                                                                      [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
                source_homogeneous_matrix = make_homogeneous_matrix([self.source_odom.pose.pose.position.x, self.source_odom.pose.pose.position.y, self.source_odom.pose.pose.position.z],
                                                                    [self.source_odom.pose.pose.orientation.x, self.source_odom.pose.pose.orientation.y, self.source_odom.pose.pose.orientation.z, self.source_odom.pose.pose.orientation.w])
                # Hnew = Hold * T -> T = Hold^-1 * Hnew
                self.offset_homogeneous_matrix = numpy.dot(numpy.linalg.inv(source_homogeneous_matrix), new_pose_homogeneous_matrix) # self.odom.header.stamp is assumed to be same as self.source_odom.header.stamp
                
    def calculate_mean_and_covariance(self, current_pose, feedback_pose):
        sources = [current_pose, feedback_pose]
        means = []
        covs = []
        for src in sources:
            euler = tf.transformations.euler_from_quaternion([src.pose.orientation.x, src.pose.orientation.y,
                                                              src.pose.orientation.z, src.pose.orientation.w])
            means.append(numpy.array([[src.pose.position.x],
                                      [src.pose.position.y],
                                      [src.pose.position.z],
                                      [euler[0]],
                                      [euler[1]],
                                      [euler[2]]])) # 6d column vector
            covs.append(numpy.mat(src.covariance).reshape(6, 6))
    
        # Calculate new state by most likelihood method:
        # sigma_new = (sigma_0^-1 + sigma_1^-1)^-1
        # x_new = sigma_new * (sigma_0^-1 * x_0 + sigma_1^-1 * x_1)
        # Less inverse form (same as above):
        # sigma__new = sigma_1 - sigma_1 * (sigma_1 + sigma_2)^-1 * sigma_1
        # x_new = x1 + sigma_1*(sigma_1+sigma_2)^-1*(x_2-x_1)
        # cf. "Distributed sensor fusion for object position estimation by multi-robot systems"
        cov0_inv_sum_covs = numpy.dot(covs[0], numpy.linalg.inv(covs[0] + covs[1]))
        new_cov = covs[0] - numpy.dot(cov0_inv_sum_covs, covs[0])        
        new_mean = means[0] + numpy.dot(cov0_inv_sum_covs, means[1] - means[0])
        return numpy.array(new_mean).reshape(-1,).tolist(), numpy.array(new_cov).reshape(-1,).tolist() # return list

    def check_feedback_time(self):
        time_from_prev_feedback = (self.odom.header.stamp - self.prev_feedback_time).to_sec()
        if self.max_feedback_time > 0 and time_from_prev_feedback > self.max_feedback_time:
            rospy.loginfo("%s: Feedback time is exceeded. %f > %f", rospy.get_name(), time_from_prev_feedback, self.max_feedback_time)
            return True
        else:
            return False

    def check_covariance(self, odom):
        for cov in odom.pose.covariance:
            if cov > self.force_feedback_sigma ** 2:
                rospy.loginfo("%s: Covariance exceeds limitation. %f > %f", rospy.get_name(), cov, self.force_feedback_sigma)
                return True
        return False

    def check_distribution_difference(self, nearest_odom, feedback_odom):
        def make_pose_set(odom):
            odom_euler = tf.transformations.euler_from_quaternion((odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                                                   odom.pose.pose.orientation.z, odom.pose.pose.orientation.w))
            odom_pose_list = [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
                              odom_euler[0], odom_euler[1], odom_euler[2]]
            odom_cov_matrix = numpy.matrix(odom.pose.covariance).reshape(6, 6)
            return odom_pose_list, odom_cov_matrix
        nearest_odom_pose, nearest_odom_cov_matrix = make_pose_set(nearest_odom)
        feedback_odom_pose, feedback_odom_cov_matrix = make_pose_set(feedback_odom)
        for i in range(6):
            if abs(nearest_odom_pose[i] - feedback_odom_pose[i]) > 3 * numpy.sqrt(nearest_odom_cov_matrix[i, i]) and numpy.sqrt(nearest_odom_cov_matrix[i, i]) > self.distribution_feedback_minimum_sigma:
                rospy.loginfo("%s: Pose difference is larger than original sigma * 3. %f > %f (> %f)",
                              rospy.get_name(), abs(nearest_odom_pose[i] - feedback_odom_pose[i]), numpy.sqrt(nearest_odom_cov_matrix[i, i]),
                              self.distribution_feedback_minimum_sigma)
                return True
        return False
            
    def publish_odometry(self):
        self.pub.publish(self.odom)
        self.odom_history.append(copy.deepcopy(self.odom))

    def calculate_odometry(self, odom, source_odom):
        result_odom = copy.deepcopy(source_odom)
        result_odom.header.frame_id = self.odom_frame
        result_odom.child_frame_id = self.base_link_frame

        # consider only pose because twist is local and copied from source_odom
        position = [source_odom.pose.pose.position.x, source_odom.pose.pose.position.y, source_odom.pose.pose.position.z]
        orientation = [source_odom.pose.pose.orientation.x, source_odom.pose.pose.orientation.y, source_odom.pose.pose.orientation.z, source_odom.pose.pose.orientation.w]

        # calculate pose (use odometry source)
        source_homogeneous_matrix = make_homogeneous_matrix(position, orientation)
        result_homogeneous_matrix = numpy.dot(source_homogeneous_matrix, self.offset_homogeneous_matrix)
        result_odom.pose.pose.position = Point(*list(result_homogeneous_matrix[:3, 3]))
        result_odom.pose.pose.orientation = Quaternion(*list(tf.transformations.quaternion_from_matrix(result_homogeneous_matrix)))

        # calculate pose covariance (do not use odometry source)
        if self.odom != None:
            result_odom.pose.covariance =  self.odom.pose.covariance # do not use source_odom covariance in pose
            dt = (self.source_odom.header.stamp - self.odom.header.stamp).to_sec()
        else:
            # initial covariance of pose is defined as same value of source_odom
            dt = 0.0
        global_twist_covariance = transform_local_twist_covariance_to_global(result_odom.pose.pose, result_odom.twist.covariance)
        result_odom.pose.covariance = update_pose_covariance(result_odom.pose.covariance, global_twist_covariance, dt)
        self.odom = result_odom

    def update_twist(self, twist, new_twist):
        twist.twist = new_twist.twist
