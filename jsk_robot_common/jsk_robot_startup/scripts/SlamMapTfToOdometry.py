#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, TwistWithCovariance, Twist, Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
import tf
import numpy

class SlamMapTfToOdometry:
    def __init__(self):
        rospy.init_node("SlamMapTfToOdometry", anonymous=True)
        self.pub = rospy.Publisher("~output", Odometry, queue_size=10)
        self.map_frame = rospy.get_param("~map_frame", "map")
        # self.base_frame = rospy.get_param("~base_frame", "BODY")
        self.rate = rospy.get_param("~rate", 10)
        self.listener = tf.TransformListener()
        self.r = rospy.Rate(self.rate) # 10hz
        self.is_new_map = False
        self.base_odom = None
        self.map_sub = rospy.Subscriber("~map", OccupancyGrid, self.map_callback, queue_size = 10)
        self.base_odom_sub = rospy.Subscriber("~base_odom", Odometry, self.base_odom_callback, queue_size = 10)

    def execute(self):
        while not rospy.is_shutdown():
            self.publish_pose_stamped()
            self.r.sleep()

    def map_callback(self, msg):
        self.is_new_map = True

    def base_odom_callback(self, msg):
        self.base_odom = msg

    def publish_pose_stamped(self):
        if self.base_odom == None:
            return
        try:
            (trans,rot) = self.listener.lookupTransform(self.map_frame, self.base_odom.child_frame_id, rospy.Time(0)) # current map->base transform
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        pub_msg = Odometry()
        pub_msg.pose.pose.position.x = trans[0]
        pub_msg.pose.pose.position.y = trans[1]
        pub_msg.pose.pose.position.z = trans[2]
        pub_msg.pose.pose.orientation.x = rot[0]
        pub_msg.pose.pose.orientation.y = rot[1]
        pub_msg.pose.pose.orientation.z = rot[2]
        pub_msg.pose.pose.orientation.w = rot[3]
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.header.frame_id = self.map_frame
        pub_msg.child_frame_id = self.base_odom.child_frame_id        

        # calculate covariance
        # use covariance of base_odom.pose and only transform it to new coords from tf
        # TODO: calc covariance in correct way, but what is that ...?
        base_odom_homogeneous_matrix = self.make_homogeneous_matrix([self.base_odom.pose.pose.position.x, self.base_odom.pose.pose.position.y, self.base_odom.pose.pose.position.z],
                                                                    [self.base_odom.pose.pose.orientation.x, self.base_odom.pose.pose.orientation.y, self.base_odom.pose.pose.orientation.z, self.base_odom.pose.pose.orientation.w])
        new_odom_homogeneous_matrix = self.make_homogeneous_matrix(trans, rot)
        base_to_new_transform = numpy.dot(base_odom_homogeneous_matrix, numpy.linalg.inv(new_odom_homogeneous_matrix))
        rotation_matrix = base_to_new_transform[:3, :3]
        original_pose_cov_matrix = numpy.matrix(self.base_odom.pose.covariance).reshape(6, 6)
        pose_cov_matrix = numpy.zeros((6, 6))
        pose_cov_matrix[:3, :3] = (rotation_matrix.T).dot(original_pose_cov_matrix[:3, :3].dot(rotation_matrix))
        pose_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(original_pose_cov_matrix[3:6, 3:6].dot(rotation_matrix))
        pub_msg.pose.covariance = numpy.array(pose_cov_matrix).reshape(-1,).tolist()

        # twist is local and transform same as covariance
        new_velocity = numpy.dot(rotation_matrix, numpy.array([[self.base_odom.twist.twist.linear.x],
                                                               [self.base_odom.twist.twist.linear.y],
                                                               [self.base_odom.twist.twist.linear.z]]))
        new_omega = numpy.dot(rotation_matrix, numpy.array([[self.base_odom.twist.twist.angular.x],
                                                            [self.base_odom.twist.twist.angular.y],
                                                            [self.base_odom.twist.twist.angular.z]]))
        original_twist_cov_matrix = numpy.matrix(self.base_odom.twist.covariance).reshape(6, 6)
        twist_cov_matrix = numpy.zeros((6, 6))
        twist_cov_matrix[:3, :3] = (rotation_matrix.T).dot(original_twist_cov_matrix[:3, :3].dot(rotation_matrix))
        twist_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(original_twist_cov_matrix[3:6, 3:6].dot(rotation_matrix))
        pub_msg.twist = TwistWithCovariance(Twist(Vector3(*new_velocity[:, 0]), Vector3(*new_omega[:, 0])), twist_cov_matrix.reshape(-1,).tolist())
        
        self.pub.publish(pub_msg)

    def make_homogeneous_matrix(self, trans, rot):
        homogeneous_matrix = tf.transformations.quaternion_matrix(rot)
        homogeneous_matrix[:3, 3] = numpy.array(trans).reshape(1, 3)
        return homogeneous_matrix

if __name__ == '__main__':
    try:
        node = SlamMapTfToOdometry()
        node.execute()
    except rospy.ROSInterruptException: pass
