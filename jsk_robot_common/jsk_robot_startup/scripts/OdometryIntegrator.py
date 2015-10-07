#! /usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
import numpy
import tf
import threading
import sys
import itertools

# input assumption:
# pose is described in each odom coordinate
# twist is described in base_link_frame coordinate same as ~base_link_frame param
# tf of each odom to base_link_frame is enabled

class OdometryIntegrator:
    def __init__(self):
        rospy.init_node("OdometryIntegrator", anonymous=True)
        self.odoms = [None, None]
        self.result_odom = None
        self.lock = threading.Lock()
        self.pub = rospy.Publisher("~output", Odometry, queue_size=10)
        self.sub = rospy.Subscriber("~source_odom_0", Odometry, self.callback, 0)
        self.sub = rospy.Subscriber("~source_odom_1", Odometry, self.callback, 1)
        self.rate = rospy.get_param("~rate", 100)
        self.r = rospy.Rate(self.rate)
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.invert_tf = rospy.get_param("~invert_tf", True)
        self.odom_frame = rospy.get_param("~odom_frame", "integrated_odom")
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.broadcast = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.time_threshould = float(rospy.get_param("~time_threshould", 0.1)) # [sec]

    def execute(self):
        while not rospy.is_shutdown():
            with self.lock:
                self.integrate_odom()
                if self.publish_tf:
                    self.broadcast_transform()
            self.r.sleep()

    def callback(self, msg, index):
        if index < len(self.odoms):
            with self.lock:
                self.odoms[index] = msg

    def integrate_odom(self):
        poses = []
        pose_covs = []
        twists = []
        twist_covs = []

        # check initialization
        for odom in self.odoms:
            if not odom:
                rospy.logwarn("odometry source cannot be subscribed")
                return
        
        # check timestamp
        for odom_pair in list(itertools.combinations(self.odoms, 2)):
            time_diff = abs((odom_pair[0].header.stamp - odom_pair[1].header.stamp).to_sec())
            if time_diff > self.time_threshould:
                rospy.logwarn("odometry source timestamp is too distant: %f [sec]", time_diff)
                return

        # adjust timestamps of odom sources
        # self.odoms.sort(cmp = lambda x, y: cmp(x.header.stamp, y.header.stamp))
        current_time = rospy.Time.now()
        for odom in self.odoms:
            dt = (current_time - odom.header.stamp).to_sec()
            self.update_odom_pose(odom, dt)

        # make state vector and covariance matrix for pose and twist from odometry source
        for i in range(len(self.odoms)):
            euler = tf.transformations.euler_from_quaternion([self.odoms[i].pose.pose.orientation.x, self.odoms[i].pose.pose.orientation.y,
                                                              self.odoms[i].pose.pose.orientation.z, self.odoms[i].pose.pose.orientation.w])
            poses.append(numpy.array([[self.odoms[i].pose.pose.position.x],
                                      [self.odoms[i].pose.pose.position.y],
                                      [self.odoms[i].pose.pose.position.z],
                                      [euler[0]],
                                      [euler[1]],
                                      [euler[2]]])) # 6d column vector
            pose_covs.append(numpy.mat(self.odoms[i].pose.covariance).reshape(6, 6))
            twists.append(numpy.array([[self.odoms[i].twist.twist.linear.x],
                                       [self.odoms[i].twist.twist.linear.y],
                                       [self.odoms[i].twist.twist.linear.z],
                                       [self.odoms[i].twist.twist.angular.x],
                                       [self.odoms[i].twist.twist.angular.y],
                                       [self.odoms[i].twist.twist.angular.z]])) # 6d column vector
            twist_covs.append(numpy.mat(self.odoms[i].twist.covariance).reshape(6, 6))
           
        new_pose, new_pose_cov = self.calculate_mean_and_covariance(poses, pose_covs) # assumed to be described in each odom frame
        new_twist, new_twist_cov = self.calculate_mean_and_covariance(twists, twist_covs) # assumed to be described in base_link_frame

        # publish integrated odometry
        self.result_odom = Odometry()
        self.result_odom.header.stamp = current_time
        self.result_odom.header.frame_id = self.odom_frame
        self.result_odom.child_frame_id = self.base_link_frame
        quat = tf.transformations.quaternion_from_euler(*new_pose[3:6])
        self.result_odom.pose.pose.position = Point(*new_pose[0:3])
        self.result_odom.pose.pose.orientation = Quaternion(*quat)
        self.result_odom.pose.covariance = numpy.array(new_pose_cov).reshape(-1,).tolist()
        self.result_odom.twist.twist.linear = Vector3(*new_twist[0:3])
        self.result_odom.twist.twist.angular = Vector3(*new_twist[3:6])
        self.result_odom.twist.covariance = numpy.array(new_twist_cov).reshape(-1,).tolist()
        self.pub.publish(self.result_odom)

    def calculate_mean_and_covariance(self, means, covs):
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
        return new_mean, new_cov

    def update_odom_pose(self, odom, dt):
        try:
            (trans,rot) = self.listener.lookupTransform(odom.header.frame_id, odom.child_frame_id, odom.header.stamp)
        except:
            try:
                rospy.logwarn("timestamp %f of tf (%s to %s) is not correct. use rospy.Time(0).",  odom.header.stamp.to_sec(), odom.header.frame_id, odom.child_frame_id)
                (trans,rot) = self.listener.lookupTransform(odom.header.frame_id, odom.child_frame_id, rospy.Time(0)) # todo: lookup odom.header.stamp
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("failed to solve tf: %s to %s", odom.header.frame_id, odom.child_frame_id)
                return
        rotation_matrix = tf.transformations.quaternion_matrix(rot)[:3, :3]
        global_velocity = numpy.dot(rotation_matrix, numpy.array([[odom.twist.twist.linear.x],
                                                                  [odom.twist.twist.linear.y],
                                                                  [odom.twist.twist.linear.z]]))
        global_omega = numpy.dot(rotation_matrix, numpy.array([[odom.twist.twist.angular.x],
                                                               [odom.twist.twist.angular.y],
                                                               [odom.twist.twist.angular.z]]))
        odom.pose.pose.position.x += global_velocity[0, 0] * dt
        odom.pose.pose.position.y += global_velocity[1, 0] * dt
        odom.pose.pose.position.z += global_velocity[2, 0] * dt
        euler = list(tf.transformations.euler_from_quaternion((odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                                               odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)))
        euler[0] += global_omega[0, 0] * dt
        euler[1] += global_omega[1, 0] * dt
        euler[2] += global_omega[2, 0] * dt
        quat = tf.transformations.quaternion_from_euler(*euler)
        odom.pose.pose.orientation = Quaternion(*quat)

    def broadcast_transform(self):
        if not self.result_odom:
            return
        position = [self.result_odom.pose.pose.position.x, self.result_odom.pose.pose.position.y, self.result_odom.pose.pose.position.z]
        orientation = [self.result_odom.pose.pose.orientation.x, self.result_odom.pose.pose.orientation.y, self.result_odom.pose.pose.orientation.z, self.result_odom.pose.pose.orientation.w]
        if self.invert_tf:
            homogeneous_matrix = tf.transformations.quaternion_matrix(orientation)
            homogeneous_matrix[:3, 3] = numpy.array(position).reshape(1, 3)
            homogeneous_matrix_inv = numpy.linalg.inv(homogeneous_matrix)
            position = list(homogeneous_matrix_inv[:3, 3])
            orientation = list(tf.transformations.quaternion_from_matrix(homogeneous_matrix_inv))
            parent_frame = self.base_link_frame
            target_frame = self.odom_frame
        else:
            parent_frame = self.odom_frame
            target_frame = self.base_link_frame
        self.broadcast.sendTransform(position, orientation, rospy.Time.now(), target_frame, parent_frame)
        
if __name__ == '__main__':
    try:
        node = OdometryIntegrator()
        node.execute()
    except rospy.ROSInterruptException: pass
