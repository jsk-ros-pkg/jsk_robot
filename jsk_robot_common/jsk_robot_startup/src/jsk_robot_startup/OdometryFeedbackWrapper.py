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

class OdometryFeedbackWrapper(object):
    def __init__(self):
        rospy.init_node("OdometryFeedbackWrapper", anonymous=True)
        self.rate = float(rospy.get_param("~rate", 100))
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.invert_tf = rospy.get_param("~invert_tf", True)
        self.odom_frame = rospy.get_param("~odom_frame", "feedback_odom")
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.max_feedback_time = rospy.get_param("~max_feedback_time", 60)
        self.twist_proportional_sigma = rospy.get_param("~twist_proportional_sigma", False)
        self.broadcast = tf.TransformBroadcaster()
        self.listener = tf.TransformListener(True, rospy.Duration(self.max_feedback_time + 10)) # 10[sec] is safety mergin for feedback
        self.odom = None # belief of this wrapper
        self.feedback_odom = None
        self.source_odom = None
        self.offset_homogeneous_matrix = None
        self.dt = 0.0
        self.prev_feedback_time = rospy.Time.now()
        self.r = rospy.Rate(self.rate)
        self.lock = threading.Lock()
        self.odom_history = []
        self.v_sigma = [rospy.get_param("~sigma_x", 0.05),
                        rospy.get_param("~sigma_y", 0.1),
                        rospy.get_param("~sigma_z", 0.0001),
                        rospy.get_param("~sigma_roll", 0.0001),
                        rospy.get_param("~sigma_pitch", 0.0001),
                        rospy.get_param("~sigma_yaw", 0.01)]
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
        with self.lock:
            for i, sigma in enumerate(["sigma_x", "sigma_y", "sigma_z", "sigma_roll", "sigma_pitch", "sigma_yaw"]):
                self.v_sigma[i] = config[sigma]
        rospy.loginfo("[%s]" + "velocity sigma updated: x: {0}, y: {1}, z: {2}, roll: {3}, pitch: {4}, yaw: {5}".format(*self.v_sigma), rospy.get_name())
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
                self.broadcast_transform()

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
            self.update_pose(nearest_odom.pose, nearest_odom.twist,
                             nearest_odom.header.frame_id, nearest_odom.child_frame_id,
                             nearest_odom.header.stamp, nearest_dt)
            enable_feedback = self.check_covariance(nearest_odom) or self.check_distribution_difference(nearest_odom, self.feedback_odom) or self.check_feedback_time()
            # update feedback_odom to approximate odom at current timestamp using previsous velocities
            if enable_feedback:
                rospy.loginfo("%s: Feedback enabled.", rospy.get_name())
                self.feedback_odom = msg
                # adjust timestamp of self.feedback_odom to current self.odom
                for hist in self.odom_history:
                    dt = (hist.header.stamp - self.feedback_odom.header.stamp).to_sec()
                    if dt > 0.0:
                        # update pose and twist according to the history
                        self.update_twist(self.feedback_odom.twist, hist.twist)
                        self.update_pose(self.feedback_odom.pose,
                                         hist.twist, self.feedback_odom.header.frame_id, hist.child_frame_id,
                                         hist.header.stamp, dt) # update feedback_odom according to twist of hist
                        # update covariance
                        # this wrapper do not upgrade twist.covariance to trust feedback_odom.covariance
                        self.update_twist_covariance(self.feedback_odom.twist)
                        self.update_pose_covariance(self.feedback_odom.pose, self.feedback_odom.twist,
                                                    self.feedback_odom.header.frame_id, hist.child_frame_id,
                                                    hist.header.stamp, dt)
                        self.feedback_odom.header.stamp = hist.header.stamp
                dt = (self.odom.header.stamp - self.feedback_odom.header.stamp).to_sec()                        
                self.update_pose(self.feedback_odom.pose, self.feedback_odom.twist,
                                 self.feedback_odom.header.frame_id, self.feedback_odom.child_frame_id,
                                 self.feedback_odom.header.stamp, dt)
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
                new_pose_homogeneous_matrix = self.make_homogeneous_matrix([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z],
                                                                           [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
                source_homogeneous_matrix = self.make_homogeneous_matrix([self.source_odom.pose.pose.position.x, self.source_odom.pose.pose.position.y, self.source_odom.pose.pose.position.z],
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
        if time_from_prev_feedback > self.max_feedback_time:
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

        # consider only pose because twist is local
        position = [source_odom.pose.pose.position.x, source_odom.pose.pose.position.y, source_odom.pose.pose.position.z]
        orientation = [source_odom.pose.pose.orientation.x, source_odom.pose.pose.orientation.y, source_odom.pose.pose.orientation.z, source_odom.pose.pose.orientation.w]

        # calculate pose (use odometry source)
        source_homogeneous_matrix = self.make_homogeneous_matrix(position, orientation)
        result_homogeneous_matrix = numpy.dot(source_homogeneous_matrix, self.offset_homogeneous_matrix)
        result_odom.pose.pose.position = Point(*list(result_homogeneous_matrix[:3, 3]))
        result_odom.pose.pose.orientation = Quaternion(*list(tf.transformations.quaternion_from_matrix(result_homogeneous_matrix)))

        # calculate covariance (do not use odometry source)
        if self.odom != None:
            result_odom.pose.covariance =  self.odom.pose.covariance # do not use source_odom covariance in pose
            dt = (self.source_odom.header.stamp - self.odom.header.stamp).to_sec()
        else:
            # initial covariance of pose is defined as same value of source_odom
            dt = 0.0
        self.update_pose_covariance(result_odom.pose, result_odom.twist, result_odom.header.frame_id, result_odom.child_frame_id, result_odom.header.stamp, dt)
        self.odom = result_odom

    def update_twist(self, twist, new_twist):
        twist.twist = new_twist.twist

    def convert_local_twist_to_global_twist(self, local_twist, pose_frame, twist_frame, stamp):
        try:
            (trans,rot) = self.listener.lookupTransform(pose_frame, twist_frame, stamp)
        except:
            try:
                # rospy.loginfo("timestamp %f of tf (%s to %s) is not correct. use rospy.Time(0).",  stamp.to_sec(), pose_frame, twist_frame)
                (trans,rot) = self.listener.lookupTransform(pose_frame, twist_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("failed to solve tf: %s to %s", pose_frame, twist_frame)
                return None
        rotation_matrix = tf.transformations.quaternion_matrix(rot)[:3, :3]
        global_velocity = numpy.dot(rotation_matrix, numpy.array([[local_twist.twist.linear.x],
                                                                  [local_twist.twist.linear.y],
                                                                  [local_twist.twist.linear.z]]))
        global_omega = numpy.dot(rotation_matrix, numpy.array([[local_twist.twist.angular.x],
                                                               [local_twist.twist.angular.y],
                                                               [local_twist.twist.angular.z]]))
        return Twist(Vector3(*global_velocity[:3, 0]), Vector3(*global_omega[:3, 0]))

    def update_pose(self, pose, twist, pose_frame, twist_frame, stamp, dt):
        global_twist = self.convert_local_twist_to_global_twist(twist, pose_frame, twist_frame, stamp)
        # calculate trapezoidal integration
        pose.pose.position.x += global_twist.linear.x * dt
        pose.pose.position.y += global_twist.linear.y * dt
        pose.pose.position.z += global_twist.linear.z * dt
        pose.pose.orientation = self.calculate_quaternion_transform(pose.pose.orientation, twist.twist.angular, dt)

    def calculate_quaternion_transform(self, orientation, angular, dt): # angular is assumed to be global
        # quaternion calculation
        quat_vec = numpy.array([[orientation.x],
                                [orientation.y],
                                [orientation.z],
                                [orientation.w]])
        # skew_omega = numpy.matrix([[0, angular.z, -angular.y, angular.x],
        #                            [-angular.z, 0, angular.x, angular.y],
        #                            [angular.y, -angular.x, 0, angular.z],
        #                            [-angular.x, -angular.y, -angular.z, 0]])
        skew_omega = numpy.matrix([[0, -angular.z, angular.y, angular.x],
                                   [angular.z, 0, -angular.x, angular.y],
                                   [-angular.y, angular.x, 0, angular.z],
                                   [-angular.x, -angular.y, -angular.z, 0]])
        new_quat_vec = quat_vec + 0.5 * numpy.dot(skew_omega, quat_vec) * dt
        norm = numpy.linalg.norm(new_quat_vec)
        if norm == 0:
            rospy.logwarn("norm of quaternion is zero")
        else:
            new_quat_vec = new_quat_vec / norm # normalize
        return Quaternion(*numpy.array(new_quat_vec).reshape(-1,).tolist())
        
    def update_twist_covariance(self, twist):
        twist_list = [twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z, twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z]
        if self.twist_proportional_sigma == True:
            current_sigma = [x * y for x, y in zip(twist_list, self.v_sigma)]
        else:
            current_sigma = self.v_sigma
        twist.covariance = numpy.diag([max(x**2, 0.001*0.001) for x in current_sigma]).reshape(-1,).tolist() # covariance should be singular

    def update_pose_covariance(self, pose, twist, pose_frame, twist_frame, stamp, dt):
        # make matirx from covariance array
        prev_pose_cov_matrix = numpy.matrix(pose.covariance).reshape(6, 6)
        twist_cov_matrix = numpy.matrix(twist.covariance).reshape(6, 6)
        # twist is described in child_frame_id coordinates
        try:
            (trans,rot) = self.listener.lookupTransform(pose_frame, twist_frame, stamp)
        except:
            try:
                # rospy.logwarn("timestamp %f of tf (%s to %s) is not correct. use rospy.Time(0).",  stamp.to_sec(), pose_frame, twist_frame)
                (trans,rot) = self.listener.lookupTransform(pose_frame, twist_frame, rospy.Time(0)) # todo: lookup odom.header.stamp
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("failed to solve tf: %s to %s", pose_frame, twist_frame)
                return
        rotation_matrix = tf.transformations.quaternion_matrix(rot)[:3, :3]
        global_twist_cov_matrix = numpy.zeros((6, 6))
        global_twist_cov_matrix[:3, :3] = (rotation_matrix.T).dot(twist_cov_matrix[:3, :3].dot(rotation_matrix))
        global_twist_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(twist_cov_matrix[3:6, 3:6].dot(rotation_matrix))
        # jacobian matrix
        # elements in pose and twist are assumed to be independent on global coordinates
        jacobi_pose = numpy.diag([1.0] * 6)
        jacobi_twist = numpy.diag([dt] * 6)
        # covariance calculation
        pose_cov_matrix = jacobi_pose.dot(prev_pose_cov_matrix.dot(jacobi_pose.T)) + jacobi_twist.dot(global_twist_cov_matrix.dot(jacobi_twist.T))
        # update covariances as array type (twist is same as before)
        pose.covariance = numpy.array(pose_cov_matrix).reshape(-1,).tolist()

    def broadcast_transform(self):
        if not self.odom:
            return
        position = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z]
        orientation = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
        if self.invert_tf:
            homogeneous_matrix = self.make_homogeneous_matrix(position, orientation)
            homogeneous_matrix_inv = numpy.linalg.inv(homogeneous_matrix)
            position = list(homogeneous_matrix_inv[:3, 3])
            orientation = list(tf.transformations.quaternion_from_matrix(homogeneous_matrix_inv))
            parent_frame = self.odom.child_frame_id
            target_frame = self.odom.header.frame_id
        else:
            parent_frame = self.odom.header.frame_id
            target_frame = self.odom.child_frame_id
        self.broadcast.sendTransform(position, orientation, self.odom.header.stamp, target_frame, parent_frame)

    def make_homogeneous_matrix(self, trans, rot):
        homogeneous_matrix = tf.transformations.quaternion_matrix(rot)
        homogeneous_matrix[:3, 3] = numpy.array(trans).reshape(1, 3)
        return homogeneous_matrix
        
