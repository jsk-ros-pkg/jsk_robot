#! /usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import tf

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
            # (trans,rot) = self.listener.lookupTransform(self.parent, '/odom', rospy.Time(0))
            # self.broadcast.sendTransform((0, 0, trans[2] + self.height), rot, rospy.Time.now(), self.frame_name, self.parent)
            # (trans,rot) = self.listener.lookupTransform(self.parent, '/odom', rospy.Time(0))
            # self.broadcast.sendTransform((0, 0, trans[2] + self.height), (0, 0, 0, 1), rospy.Time.now(), self.frame_name, self.parent)
            (trans,rot) = self.listener.lookupTransform(self.parent, '/odom', rospy.Time(0))
            rot_euler = tf.transformations.euler_from_quaternion(rot)
            target_rot = tf.transformations.quaternion_from_euler(rot_euler[0], rot_euler[1], 0.0)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

if __name__ == '__main__':
    try:
        node = ConstantHeightFramePublisher()
        node.execute()
    except rospy.ROSInterruptException: pass
