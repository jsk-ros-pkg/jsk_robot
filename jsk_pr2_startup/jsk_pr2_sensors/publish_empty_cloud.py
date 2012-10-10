#!/usr/bin/env python
import roslib; roslib.load_manifest('jsk_pr2_startup')
import rospy
import math
from struct import *
from sensor_msgs.msg import *

def talker():
    rospy.init_node('empty_cloud_publisher')
    pub = rospy.Publisher('empty_cloud', PointCloud2)
    frame_id = rospy.get_param('frame_id', '/base_laser_link')
    while not rospy.is_shutdown():
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = 72


        msg_fieldx = PointField()
        msg_fieldy = PointField()
        msg_fieldz = PointField()
        msg_fieldx.name = 'x'
        msg_fieldx.offset = 0
        msg_fieldx.datatype = 7
        msg_fieldx.count = 1
        msg_fieldy.name = 'y'
        msg_fieldy.offset = 4
        msg_fieldy.datatype = 7
        msg_fieldy.count = 1
        msg_fieldz.name = 'z'
        msg_fieldz.offset = 8
        msg_fieldz.datatype = 7
        msg_fieldz.count = 1

        msg.fields = [msg_fieldx, msg_fieldy, msg_fieldz]
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width;
        msg.is_dense = True

        for i in range(-32,32):
            x = 25*math.cos(i*0.01)
            y = 25*math.sin(i*0.01)
            z = 0.0
            msg.data += pack('<f', x)
            msg.data += pack('<f', y)
            msg.data += pack('<f', z)
            msg.data += pack('<f', 0)

        pub.publish(msg)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
