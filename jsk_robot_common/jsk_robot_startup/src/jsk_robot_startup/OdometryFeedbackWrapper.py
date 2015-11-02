#! /usr/bin/env python

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
        self.odom_init_frame = rospy.get_param("~odom_init_frame", "odom_init")
        self.odom_frame = rospy.get_param("~odom_frame", "feedback_odom")
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.max_feedback_time = rospy.get_param("~max_feedback_time", 60)
        self.twist_proportional_sigma = rospy.get_param("~twist_proportional_sigma", False)
        self.broadcast = tf.TransformBroadcaster()
        self.listener = tf.TransformListener(True, rospy.Duration(self.max_feedback_time + 10)) # 10[sec] is safety mergin for feedback
        self.odom = None # belief of this wrapper
        self.feedback_odom = None
        self.source_odom = None
        self.prev_global_twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
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
        self.feedback_enabled_sigma = rospy.get_param("~feedback_enabled_sigma", 0.5)
        self.pub = rospy.Publisher("~output", Odometry, queue_size = 100)
        self.init_signal_sub = rospy.Subscriber("~init_signal", Empty, self.init_signal_callback)
        self.source_odom_sub = rospy.Subscriber("~source_odom", Odometry, self.source_odom_callback, queue_size = 100)
        self.feedback_odom_sub = rospy.Subscriber("~feedback_odom", Odometry, self.feedback_odom_callback, queue_size = 100)
        self.reconfigure_server = Server(OdometryFeedbackWrapperReconfigureConfig, self.reconfigure_callback)
        self.initialize_odometry() # init self.odom based on init_odom_frame and base_link_frame

    def execute(self):
        while not rospy.is_shutdown():
            self.update()
            self.r.sleep()

    def reconfigure_callback(self, config, level):
        with self.lock:
            for i, sigma in enumerate(["sigma_x", "sigma_y", "sigma_z", "sigma_roll", "sigma_pitch", "sigma_yaw"]):
                self.v_sigma[i] = config[sigma]
        rospy.loginfo("[%s]" + "velocity sigma updated: x: {0}, y: {1}, z: {2}, roll: {3}, pitch: {4}, yaw: {5}".format(*self.v_sigma), rospy.get_name())
        self.feedback_enabled_sigma = config["feedback_enabled_sigma"]
        rospy.loginfo("[%s]: feedback sigma is %f", rospy.get_name(), self.feedback_enabled_sigma)
        return config

    def init_signal_callback(self, msg):
        self.initialize_odometry()

    def initialize_odometry(self):
        try:
            (trans,rot) = self.listener.lookupTransform(self.odom_init_frame, self.base_link_frame, rospy.Time(0))
        except:
            rospy.logwarn("failed to solve tf in initialize_odometry: %s to %s", self.odom_init_frame, self.base_link_frame)
            trans = [0.0, 0.0, 0.0]
            rot = [0.0, 0.0, 0.0, 1.0]
        rospy.loginfo("[%s]: initiailze odometry ", rospy.get_name())
        with self.lock:
            self.odom = Odometry()
            self.odom.pose.pose = Pose(Point(*trans), Quaternion(*rot))
            self.odom.pose.covariance = numpy.diag([0.001**2]*6).reshape(-1,).tolist() # initial covariance: 0.001[mm]**2
            self.odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            self.odom.twist.covariance = numpy.diag([0.001**2]*6).reshape(-1,).tolist()
            self.odom.header.stamp = rospy.Time.now()
            self.odom.header.frame_id = self.odom_frame
            self.odom.child_frame_id = self.base_link_frame
            self.odom_history = []
            self.prev_feedback_time = self.odom.header.stamp

    def source_odom_callback(self, msg):
        if not self.odom:
            return
        with self.lock:
            self.source_odom = msg

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
            self.update_pose(nearest_odom.pose, nearest_odom.twist, None, # use rectangular approximation for simplicity
                             nearest_odom.header.frame_id, nearest_odom.child_frame_id,
                             nearest_odom.header.stamp, nearest_dt)
            enable_feedback = self.check_covaraicne(nearest_odom) or self.check_distribution_difference(nearest_odom, self.feedback_odom)
            # update feedback_odom to approximate odom at current timestamp using previsous velocities
            if enable_feedback:
                rospy.loginfo("%s: Feedback enabled.", rospy.get_name())
                self.feedback_odom = msg
                prev_global_twist = None
                for hist in self.odom_history:
                    dt = (hist.header.stamp - self.feedback_odom.header.stamp).to_sec()
                    if dt > 0.0:
                        # update pose and twist according to the history
                        self.update_twist(self.feedback_odom.twist, hist.twist)
                        prev_global_twist = self.update_pose(self.feedback_odom.pose, hist.twist, prev_global_twist, # use rectangular approximation only first
                                                             self.feedback_odom.header.frame_id, hist.child_frame_id,
                                                             hist.header.stamp, dt) # update feedback_odom according to twist of hist
                        # update covariance
                        # this wrapper do not upgrade twist.covariance to trust feedback_odom.covariance
                        self.update_twist_covariance(self.feedback_odom.twist)
                        self.update_pose_covariance(self.feedback_odom.pose, self.feedback_odom.twist,
                                                    self.feedback_odom.header.frame_id, hist.child_frame_id,
                                                    hist.header.stamp, dt)
                        self.feedback_odom.header.stamp = hist.header.stamp
                # update self.odom by self.feedback_odom
                self.odom.header.stamp = self.feedback_odom.header.stamp
                self.odom.pose = self.feedback_odom.pose
                self.prev_feedback_time = self.feedback_odom.header.stamp
                self.odom_history = []

    def check_feedback_time(self):
        time_from_prev_feedback = (self.odom.header.stamp - self.prev_feedback_time).to_sec()
        if time_from_prev_feedback > self.max_feedback_time:
            rospy.loginfo("%s: Feedback time is exceeded. %f > %f", rospy.get_name(), time_from_prev_feedback, self.max_feedback_time)
            return True
        else:
            return False

    def check_covaraicne(self, odom):
        for cov in odom.pose.covariance:
            if cov > self.feedback_enabled_sigma ** 2:
                rospy.loginfo("%s: Covariance exceeds limitation. %f > %f", rospy.get_name(), cov, self.feedback_enabled_sigma)
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
            if abs(nearest_odom_pose[i] - feedback_odom_pose[i]) > 3 * numpy.sqrt(nearest_odom_cov_matrix[i, i]):
                rospy.loginfo("%s: Pose difference is larger than original sigma * 3. %f > %f",
                              rospy.get_name(), abs(nearest_odom_pose[i] - feedback_odom_pose[i]), numpy.sqrt(nearest_odom_cov_matrix[i, i]))
                return True
        return False
            
    def update(self):
        if not self.odom or not self.source_odom:
            return
        with self.lock:
            self.dt = (rospy.Time.now() - self.odom.header.stamp).to_sec()
            if self.dt > 0.0:
                # if self.dt > 2 * (1.0 / self.rate):
                #     rospy.logwarn("[%s]Execution time is violated. Target: %f[sec], Current: %f[sec]", rospy.get_name(), 1.0 / self.rate, self.dt)
                self.odom.header.stamp = rospy.Time.now()
                self.calc_odometry()
                self.calc_covariance()
                self.publish_odometry()
                if self.publish_tf:
                    self.broadcast_transform()

    def calc_odometry(self):
        self.update_twist(self.odom.twist, self.source_odom.twist)
        self.prev_global_twist = self.update_pose(self.odom.pose, self.odom.twist, self.prev_global_twist, self.odom.header.frame_id, self.odom.child_frame_id, rospy.Time(0), self.dt)

    def calc_covariance(self):
        self.update_twist_covariance(self.odom.twist)
        self.update_pose_covariance(self.odom.pose, self.odom.twist, self.odom.header.frame_id, self.odom.child_frame_id, rospy.Time(0), self.dt)

    def publish_odometry(self):
        self.pub.publish(self.odom)
        self.odom_history.append(copy.deepcopy(self.odom))

    def update_twist(self, twist, new_twist):
        twist.twist = new_twist.twist

    def convert_local_twist_to_global_twist(self, local_twist, pose_frame, twist_frame, stamp):
        try:
            (trans,rot) = self.listener.lookupTransform(pose_frame, twist_frame, stamp)
        except:
            try:
                rospy.logwarn("timestamp %f of tf (%s to %s) is not correct. use rospy.Time(0).",  stamp.to_sec(), pose_frame, twist_frame)
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

    def update_pose(self, pose, twist, prev_global_twist, pose_frame, twist_frame, stamp, dt):
        global_twist = self.convert_local_twist_to_global_twist(twist, pose_frame, twist_frame, stamp)
        if not global_twist:
            return prev_global_twist
        euler = list(tf.transformations.euler_from_quaternion((pose.pose.orientation.x, pose.pose.orientation.y,
                                                               pose.pose.orientation.z, pose.pose.orientation.w)))
        if not prev_global_twist: # use rectangular approximation if prev_global_twist is None
            prev_global_twist = global_twist
        # calculate trapezoidal integration
        pose.pose.position.x += 0.5 * (global_twist.linear.x + prev_global_twist.linear.x) * dt
        pose.pose.position.y += 0.5 * (global_twist.linear.y + prev_global_twist.linear.y) * dt
        pose.pose.position.z += 0.5 * (global_twist.linear.z + prev_global_twist.linear.z) * dt
        euler[0] += 0.5 * (global_twist.angular.x + prev_global_twist.angular.x) * dt
        euler[1] += 0.5 * (global_twist.angular.y + prev_global_twist.angular.y) * dt
        euler[2] += 0.5 * (global_twist.angular.z + prev_global_twist.angular.z) * dt
        quat = tf.transformations.quaternion_from_euler(*euler)
        pose.pose.orientation = Quaternion(*quat)
        return global_twist # return global twist for next iteration

    def update_twist_covariance(self, twist):
        twist_list = [twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z, twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z]
        if self.twist_proportional_sigma == True:
            current_sigma = [x * y for x, y in zip(twist_list, self.v_sigma)]
        else:
            current_sigma = self.v_sigma
        twist.covariance = numpy.diag([max(x**2, 0.001*0.001) for x in current_sigma]).reshape(-1,).tolist() # covariance should be singular

    def update_pose_covariance(self, pose, twist, pose_frame, twist_frame, stamp, dt):
        # make matirx from covarinace array
        prev_pose_cov_matrix = numpy.matrix(pose.covariance).reshape(6, 6)
        twist_cov_matrix = numpy.matrix(twist.covariance).reshape(6, 6)
        # twist is described in child_frame_id coordinates
        try:
            (trans,rot) = self.listener.lookupTransform(pose_frame, twist_frame, stamp)
        except:
            try:
                rospy.logwarn("timestamp %f of tf (%s to %s) is not correct. use rospy.Time(0).",  stamp.to_sec(), pose_frame, twist_frame)
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
        self.broadcast.sendTransform(position, orientation, self.odom.header.stamp, target_frame, parent_frame)
