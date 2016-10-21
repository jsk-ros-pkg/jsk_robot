#! /usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
import tf

class ImuRootlinkCalculator(object):
    def __init__(self):
        rospy.init_node("ImuRootlinkCalculator", anonymous=True)
        # tf parameters
        self.base_link_frame = rospy.get_param("~base_link_frame", "BODY")
        self.listener = tf.TransformListener(True, rospy.Duration(10))
        self.imu_sub = rospy.Subscriber("~input", Imu, self.imu_callback, queue_size = 1)
        self.pub = rospy.Publisher("~output", Imu, queue_size = 1)

    def execute(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def init_signal_callback(self, msg):
        time.sleep(1) # wait to update odom_init frame
        with self.lock:
            self.initial_matrix = None
            
    def imu_callback(self, msg):
        raw_imu_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        raw_imu_av = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        raw_imu_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

        try:
            self.listener.waitForTransform(self.base_link_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0)) # gyrometer->body

            trans = self.listener.asMatrix(self.base_link_frame, msg.header) # gyrometer->body
            rootlink_quat = tf.transformations.quaternion_from_matrix(trans * tf.transformations.quaternion_matrix(raw_imu_quat))
            rootlink_av = numpy.dot(trans[:3, :3], numpy.array(raw_imu_av))
            rootlink_acc = numpy.dot(trans[:3, :3], numpy.array(raw_imu_acc))
            # todo: convert covariance
        except:
            rospy.logwarn("[%s] failed to solve imu_to_base tf: %s to %s", rospy.get_name(), msg.header.frame_id, self.base_link_frame)
            return

        msg.header.frame_id = self.base_link_frame
        msg.orientation = Quaternion(*rootlink_quat)
        msg.angular_velocity = Vector3(*rootlink_av)
        msg.linear_acceleration = Vector3(*rootlink_acc)
                
        # publish
        self.pub.publish(msg)
        
if __name__ == '__main__':
    try:
        node = ImuRootlinkCalculator()
        node.execute()
    except rospy.ROSInterruptException: pass
        
