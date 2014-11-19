#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

class CameraInfoFixer:
    def __init__(self):
        topic_name = rospy.resolve_name('camera_info')
        self.sub = rospy.Subscriber(topic_name, CameraInfo, self.callback)
        self.pub = rospy.Publisher(topic_name + '_fixed', CameraInfo, queue_size=1)

    def callback(self, m):
        # https://github.com/ros-perception/vision_opencv/blob/8216fb5df7eb262601f12ac4b0c9415477717514/image_geometry/src/pinhole_camera_model.cpp#L149
        K = list(m.K)
        P = list(m.P)
        K[0*3+2] -= m.roi.x_offset
        K[1*3+2] -= m.roi.y_offset
        P[0*4+2] -= m.roi.x_offset/2
        P[1*4+2] -= m.roi.y_offset/2
        m.K = tuple(K)
        m.P = tuple(P)

        m.roi.x_offset = 0
        m.roi.y_offset = 0
        m.roi.height = m.height
        m.roi.width = m.width

        self.pub.publish(m)

if __name__ == '__main__':
    rospy.init_node('camera_info_fixer', anonymous=True)
    app = CameraInfoFixer()
    rospy.spin()
