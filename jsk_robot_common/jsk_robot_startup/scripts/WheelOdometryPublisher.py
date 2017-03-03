#! /usr/bin/env python
# license removed for brevity
import rospy
import numpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf
import sys
import threading
from jsk_robot_startup.odometry_utils import broadcast_transform

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
        self.pose = [None, None, None]
        self.velocity = [None, None, None]
        self.v_sigma = [rospy.get_param("~sigma_x", 0.05),
                        rospy.get_param("~sigma_y", 0.1),
                        rospy.get_param("~sigma_z", 0.0001),
                        rospy.get_param("~sigma_roll", 0.0001),
                        rospy.get_param("~sigma_pitch", 0.0001),
                        rospy.get_param("~sigma_yaw", 0.01)]
        self.pose_cov = numpy.zeros((6, 6)) # first covariance is zero
        self.twist_cov = numpy.diag([x**2 for x in self.v_sigma]) # constant
        self.z = None # for z evaluation
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
            self.pose[0] = msg.pose.pose.position.x
            self.pose[1] = msg.pose.pose.position.y
            self.z = msg.pose.pose.position.z
            self.pose[2] = self.orientation_to_theta(msg.pose.pose.orientation)
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
            self.calc_covariance()
            self.publish_odometry()
            if self.publish_tf:
                broadcast_transform(self.broadcast, self.odom, self.invert_tf)

    def publish_odometry(self):
        self.odom = Odometry()
        # pose are descibed in global coords
        self.odom.pose.pose.position.x = self.pose[0]
        self.odom.pose.pose.position.y = self.pose[1]
        self.odom.pose.pose.position.z = self.z
        orientation = self.theta_to_orientation(self.pose[2])
        self.odom.pose.pose.orientation.x = orientation[0]
        self.odom.pose.pose.orientation.y = orientation[1]
        self.odom.pose.pose.orientation.z = orientation[2]
        self.odom.pose.pose.orientation.w = orientation[3]
        self.odom.pose.covariance = numpy.array(self.pose_cov).reshape(-1,).tolist()
        # twists are described in local coords
        self.odom.twist.twist.linear.x = numpy.sqrt(self.velocity[0] ** 2 + self.velocity[1] ** 2)
        self.odom.twist.twist.linear.y = 0.0
        self.odom.twist.twist.linear.z = 0.0
        self.odom.twist.twist.angular.x = 0.0
        self.odom.twist.twist.angular.y = 0.0
        self.odom.twist.twist.angular.z = self.velocity[2] # vehicle assumed to be on the flat ground and angular.z is not affected by roatation in xy flat
        self.odom.twist.covariance = numpy.array(self.twist_cov).reshape(-1,).tolist()
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = self.odom_frame
        self.odom.child_frame_id = self.base_link_frame
        self.pub.publish(self.odom)

    def calc_odometry(self):
        linear = self.vel_gain * self.pedal # local linear velocity
        angular = (linear / self.wheel_base) * numpy.sin(self.handle / self.steering_gain) # local angular velocity
        self.velocity[0] = linear * numpy.cos(self.pose[2])
        self.velocity[1] = linear * numpy.sin(self.pose[2])
        self.velocity[2] = angular
        self.pose[0] += self.velocity[0] * self.dt
        self.pose[1] += self.velocity[1] * self.dt
        self.pose[2] += self.velocity[2] * self.dt

    def calc_covariance(self):
        # vehicle is assumed to be nonholonomic
        v = numpy.sqrt(self.velocity[0] ** 2 + self.velocity[1] ** 2)
        omega = self.velocity[2]
        # only use x, y, theta of covariance
        try:
            (trans,rot) = self.listener.lookupTransform(self.odom_frame, self.base_link_frame, rospy.Time(0))
        except:
            rospy.logwarn("failed to solve tf: %s to %s", self.odom_frame, self.base_link_frame)
            return
        rotation_matrix = tf.transformations.quaternion_matrix(rot)[:3, :3]
        local_twist_cov_matrix = numpy.diag([x ** 2 for x in self.v_sigma])
        global_twist_cov_matrix = numpy.zeros((6, 6))
        global_twist_cov_matrix[:3, :3] = (rotation_matrix.T).dot(local_twist_cov_matrix[:3, :3].dot(rotation_matrix))
        global_twist_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(local_twist_cov_matrix[3:6, 3:6].dot(rotation_matrix))
        tmp_twist_cov = numpy.matrix([global_twist_cov_matrix[0, 0], global_twist_cov_matrix[0, 1], global_twist_cov_matrix[0, 5],
                                      global_twist_cov_matrix[1, 0], global_twist_cov_matrix[1, 1], global_twist_cov_matrix[1, 5],
                                      global_twist_cov_matrix[5, 0], global_twist_cov_matrix[5, 1], global_twist_cov_matrix[5, 5]]).reshape(3, 3)
        for i in range(3): # trust "stop" state
            if abs(self.velocity[i]) < 0.01:
                tmp_twist_cov[i, i] = 0.01*0.01
        tmp_prev_pose_cov = numpy.matrix([self.pose_cov[0, 0], self.pose_cov[0, 1], self.pose_cov[0, 5],
                                          self.pose_cov[1, 0], self.pose_cov[1, 1], self.pose_cov[1, 5],
                                          self.pose_cov[5, 0], self.pose_cov[5, 1], self.pose_cov[5, 5]]).reshape(3, 3)
        # jacobian matrix
        jacobi_pose = numpy.matrix([1.0, 0.0, -v * numpy.sin(self.pose[2]) * self.dt,
                                    0.0, 1.0, v * numpy.cos(self.pose[2]) * self.dt,
                                    0.0, 0.0, 1.0]).reshape(3, 3)
        jacobi_twist = numpy.matrix([numpy.cos(self.pose[2]) * self.dt, -numpy.sin(self.pose[2]) * self.dt, 0.0,
                                     numpy.sin(self.pose[2]) * self.dt, numpy.cos(self.pose[2]) * self.dt, 0.0,
                                     0.0, 0.0, self.dt]).reshape(3, 3)
        # covariance calculation
        tmp_pose_cov = jacobi_pose.dot(tmp_prev_pose_cov.dot(jacobi_pose.T)) + jacobi_twist.dot(tmp_twist_cov.dot(jacobi_twist.T))
        self.pose_cov = numpy.matrix([tmp_pose_cov[0, 0], tmp_pose_cov[0, 1], 0.0, 0.0, 0.0, tmp_pose_cov[0, 2],
                                      tmp_pose_cov[1, 0], tmp_pose_cov[1, 1], 0.0, 0.0, 0.0, tmp_pose_cov[1, 2],
                                      0.0, 0.0, 0.01 * 0.01,  0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.01 * 0.01,  0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.01 * 0.01,  0.0,
                                      tmp_pose_cov[2, 0], tmp_pose_cov[2, 1], 0.0, 0.0, 0.0, tmp_pose_cov[2, 2]]).reshape(6, 6)
        self.twist_cov = local_twist_cov_matrix

if __name__ == '__main__':
    try:
        node = WheelOdometryPublisher()
        node.execute()
    except rospy.ROSInterruptException: pass
