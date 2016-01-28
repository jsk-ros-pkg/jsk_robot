#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Transform
import tf
from tf.msg import tfMessage
import numpy
import threading
import copy

class OdometryTfBroadcaster:
    def __init__(self):
        rospy.init_node("OdometryTfBroadcaster", anonymous=True)
        self.map_to_prt_sub = rospy.Subscriber("~map_to_particle", TransformStamped, self.append_transform_stamped_to_buffer, queue_size = 1)
        self.prt_to_odom_sub = rospy.Subscriber("~particle_to_odom", TransformStamped, self.append_transform_stamped_to_buffer, queue_size = 1)
        self.base_to_scan_sub = rospy.Subscriber("~base_to_scan", TransformStamped, self.append_transform_stamped_to_buffer, queue_size = 1)
        self.tf_lock = threading.Lock()
        self.tf_buffer = []
        self.rate = rospy.get_param("~rate", 50)
        self.r = rospy.Rate(self.rate)
        self.broadcaster = rospy.Publisher("/tf", tfMessage, queue_size = 100) # Make publisher for tfMessage because tf.broadcaster in python cannot receive transfomation msg list

    def execute(self):
        while not rospy.is_shutdown():
            with self.tf_lock:
                if len(self.tf_buffer) > 0:
                    self.tf_buffer.sort(key = lambda msg: msg.header.stamp.to_sec()) # It seems that transformations in tfMessage needs to be sorted by timestamp
                    self.broadcaster.publish(tfMessage(self.tf_buffer))
                    self.tf_buffer = []
            self.r.sleep()

    def append_transform_stamped_to_buffer(self, msg):
        with self.tf_lock:
            self.tf_buffer.append(msg)
                
if __name__ == '__main__':
    try:
        node = OdometryTfBroadcaster()
        node.execute()
    except rospy.ROSInterruptException: pass
