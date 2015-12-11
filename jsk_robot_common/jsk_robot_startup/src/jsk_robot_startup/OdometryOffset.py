#! /usr/bin/env python

import rospy
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3, TwistWithCovariance
from std_msgs.msg import Float64, Empty
import tf
import time
import threading
import copy

class OdometryOffset(object):
    def __init__(self):
        rospy.init_node("OdometryFeedbackWrapper", anonymous=True)
        # execute rate
        self.rate = float(rospy.get_param("~rate", 100))
        # tf parameters
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.invert_tf = rospy.get_param("~invert_tf", True)
        self.odom_frame = rospy.get_param("~odom_frame", "offset_odom")
        self.base_odom_frame = rospy.get_param("~base_odom_frame", "odom_init")
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.tf_duration = rospy.get_param("~tf_duration", 1)
        # for filter (only used when use_twist_filter is True)
        self.use_twist_filter = rospy.get_param("~use_twist_filter", False)
        self.filter_buffer_size = rospy.get_param("~filter_buffer_size", 5)
        self.filter_buffer = []
        # for covariance calculation (only used when calculate_covariance is True)
        self.calculate_covariance = rospy.get_param("~calculate_covariance", False)
        self.calculate_covariance = rospy.get_param("~twist_proportional_sigma", False)
        self.v_sigma = [rospy.get_param("~sigma_x", 0.05),
                        rospy.get_param("~sigma_y", 0.1),
                        rospy.get_param("~sigma_z", 0.0001),
                        rospy.get_param("~sigma_roll", 0.0001),
                        rospy.get_param("~sigma_pitch", 0.0001),
                        rospy.get_param("~sigma_yaw", 0.01)]
        self.broadcast = tf.TransformBroadcaster()
        self.listener = tf.TransformListener(True, rospy.Duration(120))
        self.r = rospy.Rate(self.rate)
        self.offset_matrix = None
        self.prev_time = rospy.Time.now()
        self.lock = threading.Lock()
        self.source_odom_sub = rospy.Subscriber("~source_odom", Odometry, self.source_odom_callback)
        self.init_signal_sub = rospy.Subscriber("~init_signal", Empty, self.init_signal_callback)
        self.pub = rospy.Publisher("~output", Odometry, queue_size = 1)

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
            self.prev_time = rospy.Time.now()
            self.filter_buffer = []
            
    def source_odom_callback(self, msg):
        with self.lock:
            if self.offset_matrix != None:
                source_odom_matrix = self.make_homogeneous_matrix([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                                                                  [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                new_odom = copy.deepcopy(msg)
                new_odom.header.frame_id = self.odom_frame
                new_odom.child_frame_id = self.base_link_frame

                # use median filter to cancel spike noise of twist when use_twist_filter is true
                if self.use_twist_filter:
                    vel = [new_odom.twist.twist.linear.x, new_odom.twist.twist.linear.y, new_odom.twist.twist.linear.z, new_odom.twist.twist.angular.x, new_odom.twist.twist.angular.y, new_odom.twist.twist.angular.z]
                    vel = self.median_filter(vel)
                    new_odom.twist.twist = Twist(Vector3(*vel[0:3]), Vector3(*vel[3: 6]))

                # overwrite twist covariance when calculate_covariance flag is True
                if self.calculate_covariance:
                    self.update_twist_covariance(new_odom.twist)
                    
                # offset coords
                new_odom_matrix = self.offset_matrix.dot(source_odom_matrix)
                new_odom.pose.pose.position = Point(*list(new_odom_matrix[:3, 3]))
                new_odom.pose.pose.orientation = Quaternion(*list(tf.transformations.quaternion_from_matrix(new_odom_matrix)))

                if self.calculate_covariance:
                    dt = (new_odom.header.stamp - self.prev_time).to_sec()
                    global_twist_with_covariance = self.transform_twist_with_covariance_to_global(new_odom.pose, new_odom.twist)
                    new_odom.pose.covariance = self.update_pose_covariance(new_odom.pose.covariance, global_twist_with_covariance.covariance, dt)
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
                    self.broadcast_transform(new_odom)

                self.prev_time = new_odom.header.stamp
                    
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

    def median_filter(self, data):
        self.filter_buffer.append(data)
        ret = numpy.median(self.filter_buffer, axis = 0)
        if len(self.filter_buffer) >= self.filter_buffer_size:
            self.filter_buffer.pop(0) # filter_buffer has at least 1 member
        return ret
        
    def update_twist_covariance(self, twist):
        twist_list = [twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z, twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z]
        if self.twist_proportional_sigma == True:
            current_sigma = [x * y for x, y in zip(twist_list, self.v_sigma)]
        else:
            current_sigma = self.v_sigma
        twist.covariance = numpy.diag([max(x**2, 0.001*0.001) for x in current_sigma]).reshape(-1,).tolist() # covariance should be singular

    def update_pose_covariance(self, pose_cov, global_twist_cov, dt):
        ret_pose_cov = []
        # make matirx from covariance array
        prev_pose_cov_matrix = numpy.matrix(pose_cov).reshape(6, 6)
        global_twist_cov_matrix = numpy.matrix(global_twist_cov).reshape(6, 6)
        # jacobian matrix
        # elements in pose and twist are assumed to be independent on global coordinates
        jacobi_pose = numpy.diag([1.0] * 6)
        jacobi_twist = numpy.diag([dt] * 6)
        # covariance calculation
        pose_cov_matrix = jacobi_pose.dot(prev_pose_cov_matrix.dot(jacobi_pose.T)) + jacobi_twist.dot(global_twist_cov_matrix.dot(jacobi_twist.T))
        # update covariances as array type (twist is same as before)
        ret_pose_cov = numpy.array(pose_cov_matrix).reshape(-1,).tolist()
        return ret_pose_cov
        
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
