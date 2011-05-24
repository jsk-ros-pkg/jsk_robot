#!/usr/bin/env python
import roslib; roslib.load_manifest('jsk_pr2_startup')
import rospy
import math
from sensor_msgs.msg import *
from geometry_msgs.msg import Point32
def talker():
    pub = rospy.Publisher('empty_cloud', PointCloud)
    rospy.init_node('empty_cloud_publisher')
    while not rospy.is_shutdown():
        msg=PointCloud()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_laser_link"
        msg.points = []
        for i in range(-200,200):
            msg.points += [Point32(x=25*math.cos(i*0.01),
                                   y=25*math.sin(i*0.01))]
        msg.channels = [ChannelFloat32(name='index',values=[0.0]),
                        ChannelFloat32(name='distances',values=[5.0]),
                        ChannelFloat32(name='stamps',values=[0.0])]
        pub.publish(msg)
        rospy.sleep(0.1)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
