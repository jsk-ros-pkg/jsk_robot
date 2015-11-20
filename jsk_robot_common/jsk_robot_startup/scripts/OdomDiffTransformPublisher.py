#! /usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry
import numpy
import tf

class OdomPublisher:
    def __init__(self):
        rospy.init_node("OdometryPublisher", anonymous=True)
        self.sub = rospy.Subscriber("~input_odom", Odometry, self.callback)
        self.target_frame = rospy.get_param("~target_frame", "ground_truth_odom")
        self.intermediate_frame = rospy.get_param("~intermediate_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "BODY")
        self.invert_tf = rospy.get_param("~invert_tf", False)
        self.listener = tf.TransformListener()
        self.broadcast = tf.TransformBroadcaster()
        self.r = rospy.Rate(30) # 30hz

    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def callback(self, msg):
        try:
            (trans,rot) = self.listener.lookupTransform(self.base_frame, self.intermediate_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("failed to solve tf: %s to %s", self.base_frame, self.intermediate_frame)
            return

        homogeneous_matrix_base_to_intermediate = tf.transformations.quaternion_matrix(rot)
        homogeneous_matrix_base_to_intermediate[:3, 3] = numpy.array(trans).reshape(1, 3)
        homogeneous_matrix_target_to_base = tf.transformations.quaternion_matrix((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        homogeneous_matrix_target_to_base[:3, 3] = numpy.array((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)).reshape(1, 3)
        homogeneous_matrix_target_to_intermediate = homogeneous_matrix_target_to_base.dot(homogeneous_matrix_base_to_intermediate)
        if self.invert_tf:
            inv_homohomogeneous_matrix_target_to_intermediate = numpy.linalg.inv(homogeneous_matrix_target_to_intermediate)
            self.broadcast.sendTransform(list(inv_homohomogeneous_matrix_target_to_intermediate[:3, 3]),
                                         list(tf.transformations.quaternion_from_matrix(inv_homohomogeneous_matrix_target_to_intermediate)),
                                         rospy.Time.now(),
                                         self.target_frame, self.intermediate_frame)
        else:
            self.broadcast.sendTransform(list(homogeneous_matrix_target_to_intermediate[:3, 3]),
                                         list(tf.transformations.quaternion_from_matrix(homogeneous_matrix_target_to_intermediate)),
                                         rospy.Time.now(),
                                         self.intermediate_frame, self.target_frame)

if __name__ == '__main__':
    try:
        node = OdomPublisher()
        node.execute()
    except rospy.ROSInterruptException: pass
