#! /usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
import tf

class ImuRootlinkCalculator(object):
    def __init__(self):
        rospy.init_node("ImuRootlinkCalculator", anonymous=True)
        # tf parameters
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.listener = tf.TransformListener(True, rospy.Duration(10))
        self.imu_sub = rospy.Subscriber("~input", Imu, self.imu_callback, queue_size = 10)
        self.pub = rospy.Publisher("~output", Imu, queue_size = 1)

    def execute(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def init_signal_callback(self, msg):
        time.sleep(1) # wait to update odom_init frame
        with self.lock:
            self.initial_matrix = None
            
    def imu_callback(self, msg):
        try:
            self.listener.waitForTransform(msg.header.frame_id, self.base_link_frame, msg.header.stamp, rospy.Duration(1.0))
            (trans,rot) = self.listener.lookupTransform(msg.header.frame_id, self.base_link_frame, msg.header.stamp)
        except:
            try:
                rospy.logwarn("[%s] failed to solve imu_to_base tf in %f. Use rospy.Time(0): %s to %s", rospy.get_name(), stamp.to_sec(), msg.header.frame_id, self.base_link_frame)
                (trans,rot) = self.listener.lookupTransform(msg.header.frame_id, self.base_link_frame, rospy.Time(0))
            except:
                rospy.logwarn("[%s] failed to solve imu_to_base tf: %s to %s", rospy.get_name(), msg.header.frame_id, self.base_link_frame)
                return

        # only convert orientation because acceleration and angular velocity are not used in ParticleOdometry
        imu_rotation = tf.transformations.quaternion_matrix(rot)
        imu_matrix = tf.transformations.quaternion_matrix([msg.orientation.x, msg.orientation.y,
                                                           msg.orientation.z, msg.orientation.w])
        imu_rootlink_quat = tf.transformations.quaternion_from_matrix(numpy.dot(imu_rotation, imu_matrix)) # base_link relative orientation
        msg.orientation = Quaternion(*imu_rootlink_quat)
        msg.header.frame_id = self.base_link_frame
        
        # publish
        self.pub.publish(msg)
        
if __name__ == '__main__':
    try:
        node = ImuRootlinkCalculator()
        node.execute()
    except rospy.ROSInterruptException: pass
        
