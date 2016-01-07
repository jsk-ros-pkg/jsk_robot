#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Transform
import tf
from tf.msg import tfMessage
import numpy
import threading

class OdometryTfBroadcaster:
    def __init__(self):
        rospy.init_node("OdometryTfBroadcaster", anonymous=True)
        self.map_to_prt_lock = threading.Lock()
        self.map_to_prt_msg = None
        self.map_to_prt_sub = rospy.Subscriber("~map_to_particle", TransformStamped, self.broadcast_tf, queue_size = 1)
        self.prt_to_odom_lock = threading.Lock()
        self.prt_to_odom_msg = None
        self.prt_to_odom_sub = rospy.Subscriber("~particle_to_odom", TransformStamped, self.broadcast_tf, queue_size = 1)
        self.base_to_scan_lock = threading.Lock()
        self.base_to_scan_msg = None
        self.base_to_scan_sub = rospy.Subscriber("~base_to_scan", TransformStamped, self.broadcast_tf, queue_size = 1)
        self.rate = rospy.get_param("~rate", 50)
        self.r = rospy.Rate(self.rate)
        self.broadcaster = rospy.Publisher("/tf", tfMessage, queue_size = 100) # Make publisher for tfMessage because tf.broadcaster in python cannot receive transfomation msg list

    def execute(self):
        tf_messages = []
        while not rospy.is_shutdown():
            with self.map_to_prt_lock:
                if self.map_to_prt_msg != None:
                    tf_messages.append(self.map_to_prt_msg)
            with self.prt_to_odom_lock:
                if self.prt_to_odom_msg != None:
                    tf_messages.append(self.prt_to_odom_msg)
            with self.base_to_scan_lock:
                if self.base_to_scan_msg != None:
                    tf_messages.append(self.base_to_scan_msg)
            if len(tf_messages) > 0:
                self.broadcaster.publish(tf_messages)
            self.r.sleep()

    def map_to_prt_callback(self, msg):
        with self.map_to_prt_lock:
            self.map_to_prt_msg = msg

    def prt_to_odom_callback(self, msg):
        with self.prt_to_odom_lock:
            self.prt_to_odom_msg = msg

    def base_to_scan_callback(self, msg):
        with self.base_to_scan_lock:
            self.base_to_scan_msg = msg
                
if __name__ == '__main__':
    try:
        node = OdometryTfBroadcaster()
        node.execute()
    except rospy.ROSInterruptException: pass
