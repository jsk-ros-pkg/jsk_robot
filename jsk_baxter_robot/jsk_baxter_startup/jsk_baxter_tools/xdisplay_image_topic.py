#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

import cv_bridge
import jsk_topic_tools
import rospy
from sensor_msgs.msg import Image


def cv_centerize(src, dst_shape):
    """Centerize image for specified image size

    @param src: image to centerize
    @param dst_shape: image shape (height, width) or (height, width, channel)
    """
    if src.shape[:2] == dst_shape[:2]:
        return src
    centerized = np.zeros(dst_shape, dtype=src.dtype)
    pad_vertical, pad_horizontal = 0, 0
    h, w = src.shape[:2]
    dst_h, dst_w = dst_shape[:2]
    if h < dst_h:
        pad_vertical = (dst_h - h) // 2
    if w < dst_w:
        pad_horizontal = (dst_w - w) // 2
    centerized[pad_vertical:pad_vertical+h,
               pad_horizontal:pad_horizontal+w] = src
    return centerized


class XdisplayImageTopic(object):

    def __init__(self, from_topic):
        self.pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=10)
        self.sub = rospy.Subscriber(from_topic, Image, self.cb)
        self.do_resize = rospy.get_param('~resize', True)
        self.do_centerize = rospy.get_param('~centerize', True)
        # JSK baxter's xdisplay is 1920x1200.
        # But default baxter's xdisplay is 1024x600.
        self.max_width = rospy.get_param('~max_width', 1920)
        self.max_height = rospy.get_param('~max_height', 1200)
        if not self.do_resize and self.do_centerize:
            jsk_topic_tools.jsk_logerr(
                "If '~centerize' is True, '~resize' must be True also."
                " Stopping centerizing.")
            self.do_centerize = False

    def cb(self, msg):
        if not self.do_resize:
            self.pub.publish(msg)
            return

        # resize image
        br = cv_bridge.CvBridge()
        img = br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = img.shape[:2]
        scale = min(1.0 * self.max_height / h, 1.0 * self.max_width / w)
        img = cv2.resize(img, None, None, fx=scale, fy=scale)

        # centerize image
        centerized_shape = (self.max_height, self.max_width, 3)
        if self.do_centerize:
            img = cv_centerize(img, centerized_shape)

        imgmsg = br.cv2_to_imgmsg(img)
        imgmsg.header = msg.header
        self.pub.publish(imgmsg)


if __name__ == '__main__':
    rospy.init_node('xdisplay_image_topic')

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('from_topic', help='Image topic to relay to xdisplay')
    args = parser.parse_args(rospy.myargv()[1:])
    from_topic = args.from_topic

    app = XdisplayImageTopic(from_topic=from_topic)
    rospy.spin()
