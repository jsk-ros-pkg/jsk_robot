#! /usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import tf
import numpy

class ConstantHeightFramePublisher:
    def __init__(self):
        rospy.init_node("ConstantHeightFramePublisher", anonymous=True)
        self.sub = rospy.Subscriber("~height", Float64, self.height_callback)
        self.broadcast = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(100)
        self.height = rospy.get_param("~height", 1.0) # [m]
        self.parent = rospy.get_param("~parent_frame", "BODY")
        self.frame_name = rospy.get_param("~frame_name", "pointcloud_to_scan_base")

    def execute(self):
        while not rospy.is_shutdown():
            self.make_constant_tf()
            self.rate.sleep()

    def height_callback(self, msg):
        self.height = msg.data

    def make_constant_tf(self):
        try:
            (trans,rot) = self.listener.lookupTransform(self.parent, '/odom', rospy.Time(0))
            # transformation: (x, y): same as parent, z: equal to height
            T = tf.transformations.quaternion_matrix(rot)
            T[:3, 3] = trans
            T_inv = tf.transformations.inverse_matrix(T)
            target_trans = T.dot(numpy.array([T_inv[:3, 3][0], T_inv[:3, 3][1], self.height, 1]))[:3]
            # rotation: (x, y): same as /odom, z: same as parent 
            rot_euler = tf.transformations.euler_from_quaternion(rot)
            target_rot = tf.transformations.quaternion_from_euler(rot_euler[0], rot_euler[1], 0.0)
            # publish tf
            self.broadcast.sendTransform(target_trans, target_rot, rospy.Time.now(), self.frame_name, self.parent)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

if __name__ == '__main__':
    try:
        node = ConstantHeightFramePublisher()
        node.execute()
    except rospy.ROSInterruptException: pass
