#! /usr/bin/env python
# license removed for brevity
import rospy
import numpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf
import sys
import threading
from jsk_robot_startup.odometry_utils import update_pose, broadcast_transform, transform_local_twist_covariance_to_global

class WheelOdometryPublisher:
    def __init__(self):
        rospy.init_node("WheelOdometryPublisher", anonymous=True)
        self.pub = rospy.Publisher("~output", Odometry, queue_size=10)
        self.broadcast = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.invert_tf = rospy.get_param("~invert_tf", True)
        self.rate = float(rospy.get_param("~rate", 100))
        self.odom_frame = rospy.get_param("~odom_frame", "wheel_odom")
        self.base_link_frame = rospy.get_param("~base_link_frame", "base_footprint") 
        self.odom = None
        self.dt = 1.0 / self.rate
        self.v_sigma = [rospy.get_param("~sigma_x", 0.05),
                        rospy.get_param("~sigma_y", 0.01),
                        rospy.get_param("~sigma_z", 0.0001),
                        rospy.get_param("~sigma_roll", 0.0001),
                        rospy.get_param("~sigma_pitch", 0.0001),
                        rospy.get_param("~sigma_yaw", 0.01)]
        self.sigma_z_pose = rospy.get_param("~sigma_z_pose", 0.01)
        self.sigma_pitch_orientation = rospy.get_param("~sigma_pitch_orientation", 0.01)
        self.sigma_roll_orientation = rospy.get_param("~sigma_roll_orientation", 0.01)
        self.v = 0.0
        self.omega = 0.0
        self.init_odom = False
        self.lock = threading.Lock()
        self.r = rospy.Rate(self.rate)
        self.velocity_sub = rospy.Subscriber("~velocity", Float64, self.velocity_callback)
        self.omega_sub = rospy.Subscriber("~omega", Float64, self.omega_callback)
        self.odom_init_sub = rospy.Subscriber("~init_odom", Odometry, self.odom_init_callback)

    def execute(self):
        while not rospy.is_shutdown():
            self.update()
            self.r.sleep()

    def odom_init_callback(self, msg):
        # initialize
        if not self.init_odom:
            self.odom = msg
            self.odom.header.frame_id = self.odom_frame
            self.odom.child_frame_id = self.base_link_frame
            self.init_odom = True

    def velocity_callback(self, msg):
        with self.lock:
            self.v = msg.data
            
    def omega_callback(self, msg):
        with self.lock:
            self.omega = msg.data

    def orientation_to_theta(self, orientation):
        euler = tf.transformations.euler_from_quaternion((orientation.x, orientation.y,
                                                          orientation.z, orientation.w))
        return euler[2]
    def theta_to_orientation(self, theta):
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        return quaternion

    def update(self):
        if not self.init_odom:
            return
        with self.lock:
            self.calc_odometry()
            self.publish_odometry()
            if self.publish_tf:
                broadcast_transform(self.broadcast, self.odom, self.invert_tf)

    def publish_odometry(self):
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = self.odom_frame
        self.odom.child_frame_id = self.base_link_frame
        self.pub.publish(self.odom)

    def calc_odometry(self, theta):
        global_twist = Twist()
        theta = self.orientation_to_theta(self.odom.pose.pose.orientation) # theta(t-1)
        
        # update pose, only consider 2d
        global_twist.linear.x = self.velocity * numpy.cos(theta)
        global_twist.linear.y = self.velocity * numpy.sin(theta)
        global_twist.linear.z = 0.0
        global_twist.angular.x = 0.0
        global_twist.angular.y = 0.0
        global_twist.angular.z = self.omega
        self.odom.pose.pose = update_pose(self.odom.pose, global_twist, self.dt)

        # update covariance, only use x, y, theta of covariance
        if abs(self.velocity) < 1e-3 and abs(self.omega) < 1e-3: # trust stop state
            local_twist_cov = numpy.diag([0.01 ** 2] * 6).reshape(-1,).tolist()
        else:
            local_twist_cov = numpy.diag([x ** 2 for x in self.v_sigma]).reshape(-1,).tolist()
        global_twist_cov = transform_local_twist_covariance_to_global(self.odom.pose.pose, local_twist_cov)
        # only use x, y, theta of covariance
        2d_twist_cov = numpy.matrix([global_twist_cov_matrix[0, 0], global_twist_cov_matrix[0, 1], global_twist_cov_matrix[0, 5],
                                     global_twist_cov_matrix[1, 0], global_twist_cov_matrix[1, 1], global_twist_cov_matrix[1, 5],
                                     global_twist_cov_matrix[5, 0], global_twist_cov_matrix[5, 1], global_twist_cov_matrix[5, 5]]).reshape(3, 3)
        # previous pose matrix
        prev_pose_cov = numpy.matrix(self.odom.pose.covariance).reshape(6, 6)
        2d_prev_pose_cov = numpy.matrix([self.pose_cov[0, 0], self.pose_cov[0, 1], self.pose_cov[0, 5],
                                         self.pose_cov[1, 0], self.pose_cov[1, 1], self.pose_cov[1, 5],
                                         self.pose_cov[5, 0], self.pose_cov[5, 1], self.pose_cov[5, 5]]).reshape(3, 3)
        # jacobian matrix
        jacobi_pose = numpy.matrix([1.0, 0.0, -self.velocity * numpy.sin(theta) * self.dt,
                                    0.0, 1.0, self.velocity * numpy.cos(theta) * self.dt,
                                    0.0, 0.0, 1.0]).reshape(3, 3)
        jacobi_twist = numpy.matrix([numpy.cos(theta) * self.dt, -numpy.sin(theta) * self.dt, 0.0,
                                     numpy.sin(theta) * self.dt, numpy.cos(theta) * self.dt, 0.0,
                                     0.0, 0.0, self.dt]).reshape(3, 3)
        # covariance calculation
        2d_pose_cov = jacobi_pose.dot(2d_prev_pose_cov.dot(jacobi_pose.T)) + jacobi_twist.dot(2d_twist_cov.dot(jacobi_twist.T))
        self.odom.pose.covariance = [2d_pose_cov[0, 0], 2d_pose_cov[0, 1], 0.0, 0.0, 0.0, 2d_pose_cov[0, 2],
                                     2d_pose_cov[1, 0], 2d_pose_cov[1, 1], 0.0, 0.0, 0.0, 2d_pose_cov[1, 2],
                                     0.0, 0.0, self.sigma_z_pose ** 2,  0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, self.sigma_roll_orientation ** 2,  0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, self.sigma_pitch_orientation ** 2,  0.0,
                                     2d_pose_cov[2, 0], 2d_pose_cov[2, 1], 0.0, 0.0, 0.0, 2d_pose_cov[2, 2]]
        self.odom.twist.covariance = local_twist_cov

if __name__ == '__main__':
    try:
        node = WheelOdometryPublisher()
        node.execute()
    except rospy.ROSInterruptException: pass
