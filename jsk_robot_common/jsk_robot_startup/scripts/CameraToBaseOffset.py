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
from jsk_robot_startup.odometry_utils import broadcast_transform, make_homogeneous_matrix

class CameraToBaseOffset(object):
    def __init__(self):
        rospy.init_node("CameraToBaseOffset", anonymous=True)
        # execute rate
        self.rate = float(rospy.get_param("~rate", 100))
        # tf parameters
        self.publish_tf = rospy.get_param("~publish_tf", True)
        if self.publish_tf:
            self.invert_tf = rospy.get_param("~invert_tf", True)
            self.broadcast = tf.TransformBroadcaster()
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.camera_frame = rospy.get_param("~camera_frame", "left_camera_optical_frame")
        self.odom_frame = rospy.get_param("~odom_frame", "viso_odom")        
        self.tf_duration = rospy.get_param("~tf_duration", 1)
        self.listener = tf.TransformListener(True, rospy.Duration(10))
        self.r = rospy.Rate(self.rate)
        self.initial_matrix = None
        self.lock = threading.Lock()
        self.source_odom_sub = rospy.Subscriber("~source_odom", Odometry, self.source_odom_callback)
        self.init_signal_sub = rospy.Subscriber("~init_signal", Empty, self.init_signal_callback)
        self.pub = rospy.Publisher("~output", Odometry, queue_size = 1)

    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def init_signal_callback(self, msg):
        time.sleep(1) # wait to update odom_init frame
        with self.lock:
            self.initial_matrix = None
            
    def source_odom_callback(self, msg):
        with self.lock:
            # calculate camera transform
            current_camera_to_base = self.calculate_camera_to_base_transform(msg.header.stamp)
            if current_camera_to_base == None:
                return

            if self.initial_matrix == None:
                self.initial_matrix = numpy.linalg.inv(current_camera_to_base)

            camera_relative_base_transformation = numpy.dot(self.initial_matrix, current_camera_to_base) # base_link transformation in camera coords
            
            # calculate offseted odometry
            source_odom_matrix = make_homogeneous_matrix([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                                                         [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            new_odom_matrix = camera_relative_base_transformation.dot(source_odom_matrix)

            # make odometry msg. twist is copied from source_odom
            new_odom = copy.deepcopy(msg)
            new_odom.header.frame_id = self.odom_frame
            new_odom.child_frame_id = self.base_link_frame
            new_odom.pose.pose.position = Point(*list(new_odom_matrix[:3, 3]))
            new_odom.pose.pose.orientation = Quaternion(*list(tf.transformations.quaternion_from_matrix(new_odom_matrix)))
            new_pose_cov_matrix = numpy.matrix(new_odom.pose.covariance).reshape(6, 6)
            rotation_matrix = camera_relative_base_transformation[:3, :3]
            new_pose_cov_matrix[:3, :3] = (rotation_matrix.T).dot(new_pose_cov_matrix[:3, :3].dot(rotation_matrix))
            new_pose_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(new_pose_cov_matrix[3:6, 3:6].dot(rotation_matrix))
            new_odom.pose.covariance = numpy.array(new_pose_cov_matrix).reshape(-1,).tolist()
                                            
            # publish
            self.pub.publish(new_odom)
            if self.publish_tf:
                broadcast_transform(self.broadcast, new_odom, self.invert_tf)

    def calculate_camera_to_base_transform(self, stamp):
        try:
            (trans,rot) = self.listener.lookupTransform(self.camera_frame, self.base_link_frame, stamp)
        except:
            try:
                rospy.logwarn("[%s] failed to solve camera_to_base tf in %f. Use rospy.Time(0): %s to %s", rospy.get_name(), stamp.to_sec(), self.camera_frame, self.base_link_frame)
                (trans,rot) = self.listener.lookupTransform(self.camera_frame, self.base_link_frame, rospy.Time(0))
            except:
                rospy.logwarn("[%s] failed to solve camera_to_base tf: %s to %s", rospy.get_name(), self.camera_frame, self.base_link_frame)
                return None
        camera_to_base_link = make_homogeneous_matrix(trans, rot) # camera -> base_link
        return camera_to_base_link
        
if __name__ == '__main__':
    try:
        node = CameraToBaseOffset()
        node.execute()
    except rospy.ROSInterruptException: pass
        
