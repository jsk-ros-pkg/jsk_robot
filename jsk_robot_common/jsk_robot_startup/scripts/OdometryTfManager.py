#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Transform
import tf
import numpy

class OdometryTfManager:
    def __init__(self):
        rospy.init_node("SlamMapTfToOdometry", anonymous=True)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.particle_frame = rospy.get_param("~particle_frame", "biped_odom_particle")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "BODY")
        self.scan_frame = rospy.get_param("~scan_frame", "pointcloud_to_laserscan_base")
        self.map_to_prt_pub = rospy.Publisher("~map_to_particle", TransformStamped, queue_size = 1)
        self.prt_to_odom_pub = rospy.Publisher("~particle_to_odom", TransformStamped, queue_size = 1)
        self.base_to_scan_pub = rospy.Publisher("~base_to_scan", TransformStamped, queue_size = 1)
        self.rate = rospy.get_param("~rate", 50)
        self.listener = tf.TransformListener()
        self.r = rospy.Rate(self.rate) # 10hz

    def execute(self):
        while not rospy.is_shutdown():
            self.publish_transform_stamped()
            self.r.sleep()

    def publish_transform_stamped(self):
        stamp = rospy.Time.now()
        map_to_prt_tf = self.make_transform_stamped(stamp, self.map_frame, self.particle_frame)
        if map_to_prt_tf != None:
            self.map_to_prt_pub.publish(map_to_prt_tf)
        prt_to_odom_tf = self.make_transform_stamped(stamp, self.particle_frame, self.odom_frame)
        if prt_to_odom_tf != None:
            self.prt_to_odom_pub.publish(prt_to_odom_tf)
        base_to_scan_tf = self.make_transform_stamped(stamp, self.base_frame, self.scan_frame)
        if base_to_scan_tf != None:
            self.base_to_scan_pub.publish(base_to_scan_tf)

    def make_transform_stamped(self, stamp, parent_frame, child_frame):
        try:
            (trans,rot) = self.listener.lookupTransform(parent_frame, child_frame, rospy.Time(0)) # current map->base transform
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("[%s]" + " Failed to solve tf: %s to %s", rospy.get_name(), parent_frame, child_frame)
            return None
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = stamp
        tf_stamped.header.frame_id = parent_frame
        tf_stamped.child_frame_id = child_frame
        tf_stamped.transform = Transform(Vector3(*trans), Quaternion(*rot))
        return tf_stamped

if __name__ == '__main__':
    try:
        node = OdometryTfManager()
        node.execute()
    except rospy.ROSInterruptException: pass
