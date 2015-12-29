#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Transform
import tf
import numpy

class OdometryTfBroadcaster:
    def __init__(self):
        rospy.init_node("OdometryTfBroadcaster", anonymous=True)
        self.map_to_prt_sub = rospy.Subscriber("~map_to_particle", TransformStamped, self.broadcast_tf, queue_size = 1)
        self.prt_to_odom_sub = rospy.Subscriber("~particle_to_odom", TransformStamped, self.broadcast_tf, queue_size = 1)
        self.rate = rospy.get_param("~rate", 50)
        self.br = tf.TransformBroadcaster()
        self.r = rospy.Rate(self.rate) # 10hz

    def execute(self):
        rospy.spin()

    # def broadcast_map_to_particle(self, msg):
    #     self.broadcast_tf(msg)

    # def broadcast_particle_to_odom(self, msg):
    #     self.broadcast_tf(msg)
        
    def broadcast_tf(self, msg):
        self.br.sendTransform((msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z),
                              (msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w),
                              msg.header.stamp, msg.child_frame_id, msg.header.frame_id)

if __name__ == '__main__':
    try:
        node = OdometryTfBroadcaster()
        node.execute()
    except rospy.ROSInterruptException: pass
