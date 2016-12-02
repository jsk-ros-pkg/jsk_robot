#!/usr/bin/env python
import roslib; roslib.load_manifest('jsk_pr2_startup')
import rospy
import math
import tf
from numpy import array,matrix
from sensor_msgs.msg import *
from geometry_msgs.msg import Point32

def abs_ray_trace (msg):
    global tf_listener, sub, pub, thre
    dummy_height = 0.2
    floor_frame_id = '/base_footprint'
    laser_frame_id = '/laser_tilt_link'

    tf_listener.waitForTransform(floor_frame_id, laser_frame_id,
                                 msg.header.stamp, rospy.Duration(0.5))
    (trans,rot) = tf_listener.lookupTransform(floor_frame_id, laser_frame_id,
                                              msg.header.stamp)
    for pt in msg.points:
        if pt.z < thre:
            alpha = - pt.z / (trans[2] - pt.z)
            pt.x = alpha * trans[0] + (1 - alpha) * pt.x
            pt.y = alpha * trans[1] + (1 - alpha) * pt.y
            pt.z = dummy_height
    pub.publish(msg)

def abs_cloud (msg):
    global pub, thre
    for p in msg.points:
        p.z = p.z if thre < p.z else abs(p.z)
    pub.publish(msg)

if __name__ == '__main__':
    global tf_listener, sub, pub, thre

    rospy.init_node('reflect_scan_publisher')
    tf_listener = tf.TransformListener()
    sub = rospy.Subscriber('input_cloud', PointCloud, abs_ray_trace)
    pub = rospy.Publisher('output_cloud', PointCloud)
    thre = rospy.get_param('~z_threshold',-0.05)

    rospy.spin()
