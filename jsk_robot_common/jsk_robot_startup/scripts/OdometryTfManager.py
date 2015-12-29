#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Transform
import tf
import numpy

class OdometryTfManager:
    def __init__(self):
        rospy.init_node("SlamMapTfToOdometry", anonymous=True)
        self.pub = rospy.Publisher("~output", Odometry, queue_size=10)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.particle_frame = rospy.get_param("~particle_frame", "biped_odom_particle")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.map_to_prt_pub = rospy.Publisher("~map_to_particle", TransformStamped, queue_size = 1)
        self.prt_to_odom_pub = rospy.Publisher("~particle_to_odom", TransformStamped, queue_size = 1)
        self.rate = rospy.get_param("~rate", 50)
        self.listener = tf.TransformListener()
        self.r = rospy.Rate(self.rate) # 10hz

    def execute(self):
        while not rospy.is_shutdown():
            self.publish_transform_stamped()
            self.r.sleep()

    def publish_pose_stamped(self):
        if self.base_odom == None:
            return
        try:
            (map_to_prt_trans,map_to_prt_rot) = self.listener.lookupTransform(self.map_frame, self.particle_frame, rospy.Time(0)) # current map->base transform
            (prt_to_odom_trans,prt_to_odom_rot) = self.listener.lookupTransform(self.particle_frame, self.odom_frame, rospy.Time(0)) # current map->base transform
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        stamp = rospy.Time.now()
        
        map_to_prt_tf = TransformStamped()
        map_to_prt_tf.header.stamp = stamp
        map_to_prt_tf.header.frame_id = self.map_frame
        map_to_prt_tf.child_frame_id = self.particle_frame
        map_to_prt_tf.transform = Transform(Vector3(*map_to_prt_trans), Quaternion(*map_to_prt_rot))
        map_to_prt_pub.publish(map_to_prt_tf)

        prt_to_odom_tf = TransformStamped()
        prt_to_odom_tf.header.stamp = stamp
        prt_to_odom_tf.header.frame_id = self.particle_frame
        prt_to_odom_tf.child_frame_id = self.odom_frame
        prt_to_odom_tf.transform = Transform(Vector3(*prt_to_odom_trans), Quaternion(*prt_to_odom_rot))

if __name__ == '__main__':
    try:
        node = OdometryTfManager()
        node.execute()
    except rospy.ROSInterruptException: pass
