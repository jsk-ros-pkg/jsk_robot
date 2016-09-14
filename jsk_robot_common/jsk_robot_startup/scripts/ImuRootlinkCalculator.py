#! /usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Quaternion, QuaternionStamped, Vector3Stamped
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
        raw_imu_quat = QuaternionStamped()
        raw_imu_quat.header = msg.header
        raw_imu_quat.quaternion = msg.orientation

        raw_imu_av = Vector3Stamped()
        raw_imu_av.header = msg.header
        raw_imu_av.vector = msg.angular_velocity

        raw_imu_acc = Vector3Stamped()
        raw_imu_acc.header = msg.header
        raw_imu_acc.vector = msg.linear_acceleration
                
        try:
            self.listener.waitForTransform(self.base_link_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0)) # gyrometer->body
            imu_rootlink_quat = self.listener.transformQuaternion(self.base_link_frame, raw_imu_quat)
            imu_rootlink_av = self.listener.transformVector3(self.base_link_frame, raw_imu_av)
            imu_rootlink_acc = self.listener.transformVector3(self.base_link_frame, raw_imu_acc)
            # todo: convert covariance
        except:
            rospy.logwarn("[%s] failed to solve imu_to_base tf: %s to %s", rospy.get_name(), msg.header.frame_id, self.base_link_frame)
            return

        msg.header = imu_rootlink_quat.header
        msg.orientation = imu_rootlink_quat.quaternion
        msg.angular_velocity = imu_rootlink_av.vector        
        msg.linear_acceleration = imu_rootlink_acc.vector
                
        # publish
        self.pub.publish(msg)
        
if __name__ == '__main__':
    try:
        node = ImuRootlinkCalculator()
        node.execute()
    except rospy.ROSInterruptException: pass
        
