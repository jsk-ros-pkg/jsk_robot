#! /usr/bin/env python
import rospy
from geometry_msgs.msg import TwistWithCovariance, Twist, Vector3, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
import tf
import numpy
from jsk_robot_startup.odometry_utils import make_homogeneous_matrix 
import threading

class SlamMapTfToOdometry:
    def __init__(self):
        rospy.init_node("SlamMapTfToOdometry", anonymous=True)
        self.pub = rospy.Publisher("~output", Odometry, queue_size=10)
        self.map_frame = rospy.get_param("~map_frame", "map")
        # self.base_frame = rospy.get_param("~base_frame", "BODY")
        self.rate = rospy.get_param("~rate", 10)
        self.pub_only_map_updated = rospy.get_param("~pub_only_map_updated", True)
        self.listener = tf.TransformListener()
        self.r = rospy.Rate(self.rate) # 10hz
        self.is_new_map = False
        self.slam_odom = None
        self.lock = threading.Lock()
        self.map_sub = rospy.Subscriber("~map", OccupancyGrid, self.map_callback, queue_size = 10)
        self.base_odom_sub = rospy.Subscriber("~base_odom", Odometry, self.base_odom_callback, queue_size = 10)

    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def map_callback(self, msg):
        if self.pub_only_map_updated and self.slam_odom != None:
            with self.lock:
                self.pub.publish(self.slam_odom)

    def base_odom_callback(self, msg):
        with self.lock:
            try:
                self.listener.waitForTransform(self.map_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(3))
                (trans,rot) = self.listener.lookupTransform(self.map_frame, msg.header.frame_id, msg.header.stamp) # current map->base_odom transform
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return
            base_odom_homogeneous_matrix = make_homogeneous_matrix([getattr(msg.pose.pose.position, attr) for attr in ["x", "y", "z"]],
                                                                   [getattr(msg.pose.pose.orientation, attr) for attr in ["x", "y", "z", "w"]]) # base_odom -> body
            map_to_base_odom_homogeneous_matrix = make_homogeneous_matrix(trans, rot) # map -> base_odom
            slam_odom_matrix = map_to_base_odom_homogeneous_matrix.dot(base_odom_homogeneous_matrix) # map -> body

            self.slam_odom = Odometry()
            self.slam_odom.pose.pose.position = Point(*list(slam_odom_matrix[:3, 3]))
            self.slam_odom.pose.pose.orientation = Quaternion(*list(tf.transformations.quaternion_from_matrix(slam_odom_matrix)))
            self.slam_odom.header.stamp = msg.header.stamp
            self.slam_odom.header.frame_id = self.map_frame
            self.slam_odom.child_frame_id = msg.child_frame_id        

            # calculate covariance
            # use covariance of base_odom.pose and only transform it to new coords from tf
            # TODO: calc covariance in correct way, but what is that ...?
            rotation_matrix = map_to_base_odom_homogeneous_matrix[:3, :3]
            original_pose_cov_matrix = numpy.matrix(msg.pose.covariance).reshape(6, 6)
            pose_cov_matrix = numpy.zeros((6, 6))
            pose_cov_matrix[:3, :3] = (rotation_matrix.T).dot(original_pose_cov_matrix[:3, :3].dot(rotation_matrix))
            pose_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(original_pose_cov_matrix[3:6, 3:6].dot(rotation_matrix))
            self.slam_odom.pose.covariance = numpy.array(pose_cov_matrix).reshape(-1,).tolist()

            # twist is local and transform same as covariance
            new_velocity = numpy.dot(rotation_matrix, numpy.array([[msg.twist.twist.linear.x],
                                                                   [msg.twist.twist.linear.y],
                                                                   [msg.twist.twist.linear.z]]))
            new_omega = numpy.dot(rotation_matrix, numpy.array([[msg.twist.twist.angular.x],
                                                                [msg.twist.twist.angular.y],
                                                                [msg.twist.twist.angular.z]]))
            original_twist_cov_matrix = numpy.matrix(msg.twist.covariance).reshape(6, 6)
            twist_cov_matrix = numpy.zeros((6, 6))
            twist_cov_matrix[:3, :3] = (rotation_matrix.T).dot(original_twist_cov_matrix[:3, :3].dot(rotation_matrix))
            twist_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(original_twist_cov_matrix[3:6, 3:6].dot(rotation_matrix))
            self.slam_odom.twist = TwistWithCovariance(Twist(Vector3(*new_velocity[:, 0]), Vector3(*new_omega[:, 0])), twist_cov_matrix.reshape(-1,).tolist())
            
            if not self.pub_only_map_updated:
                self.pub.publish(self.slam_odom)

if __name__ == '__main__':
    try:
        node = SlamMapTfToOdometry()
        node.execute()
    except rospy.ROSInterruptException: pass
