#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy

from jsk_topic_tools import ConnectionBasedTransport

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

from jsk_robot_startup.msg import RoomLight


class CheckRoomLightNode(ConnectionBasedTransport):
    def __init__(self):
        super(CheckRoomLightNode, self).__init__()
        self.bridge = CvBridge()
        self.pub = self.advertise('~output', RoomLight, queue_size=1)
        self.luminance_threshold = rospy.get_param('~luminance_threshold', 50)
        self.transport_hint = rospy.get_param('~image_transport', 'raw')
        rospy.loginfo("Using transport {}".format(self.transport_hint))

    def subscribe(self):
        if self.transport_hint == 'compressed':
            self.sub = rospy.Subscriber(
                '{}/compressed'.format(rospy.resolve_name('~input')),
                CompressedImage, self._image_cb, queue_size=1, buff_size=2**26)
        else:
            self.sub = rospy.Subscriber(
                '~input', Image, self._image_cb, queue_size=1, buff_size=2**26)

    def unsubscribe(self):
        self.sub.unregister()

    def _image_cb(self, msg):
        if self.transport_hint == 'compressed':
            encoding = msg.format.split(';')[0]
        else:
            encoding = msg.encoding
        if encoding == 'mono8':
            if self.transport_hint == 'compressed':
                np_arr = np.fromstring(msg.data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            lum_img = img
        else:
            if self.transport_hint == 'compressed':
                np_arr = np.fromstring(msg.data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                img = img[:, :, ::-1]
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            # lum = 0. 299 * R + 0.587 * G + 0.114 * B
            lum_img = 0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] \
                + 0.114 * img[:, :, 2]
        luminance = np.mean(lum_img)
        light_msg = RoomLight(header=msg.header)
        light_msg.light_on = luminance > self.luminance_threshold
        light_msg.luminance = luminance
        self.pub.publish(light_msg)


if __name__ == '__main__':
    rospy.init_node('check_room_light')
    app = CheckRoomLightNode()
    rospy.spin()
