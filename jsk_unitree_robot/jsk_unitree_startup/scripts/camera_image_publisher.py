#!/usr/bin/env python

import base64

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import cv_bridge

from paho.mqtt import client as mqtt_client


def decode_image_cv2(b64encoded):
    bin = b64encoded.split(",")[-1]
    bin = base64.b64decode(bin)
    bin = np.frombuffer(bin, np.uint8)
    img = cv2.imdecode(bin, cv2.IMREAD_COLOR)
    return img


class ImagePublisher(object):
    broker = '192.168.123.161'
    port = 1883
    topic = "vision/front_camera"

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.encoding = rospy.get_param('~encoding', 'bgr8')
        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.pub = rospy.Publisher('~output', Image, queue_size=1)
        self.pub_compressed = rospy.Publisher(
            '{}/compressed'.format(rospy.resolve_name('~output')),
            CompressedImage, queue_size=1)

        self.connect_mqtt()
        self.subscribe()
        self.client.loop_start()

    def connect_mqtt(self):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker! ({}:{})".format(self.broker, self.port))
            else:
                print("Failed to connect, return code %d\n", rc)
        self.client = mqtt_client.Client(rospy.get_name())
        self.client.on_connect = on_connect
        self.client.connect(self.broker, self.port)
        return

    def subscribe(self):
        def on_message(client, userdata, msg):
            rospy.loginfo("Received `{}` topic".format(msg.topic))

            if self.pub.get_num_connections() == 0 and self.pub_compressed.get_num_connections() == 0:
                return
            now = rospy.Time.now()
            frame = decode_image_cv2(msg.payload.decode('ascii'))
            if self.pub.get_num_connections() > 0:
                img_msg = self.bridge.cv2_to_imgmsg(
                    frame, encoding=self.encoding)
                img_msg.header.frame_id = self.frame_id
                img_msg.header.stamp = now
                self.pub.publish(img_msg)
            if self.pub_compressed.get_num_connections() > 0:
                compressed_msg = CompressedImage()
                # compressed format is separated by ';'.
                # https://github.com/ros-perception/image_transport_plugins/blob/f0afd122ed9a66ff3362dc7937e6d465e3c3ccf7/compressed_image_transport/src/compressed_publisher.cpp#L116-L128
                compressed_msg.format = '{}; {} compressed {}'.format(
                    self.encoding, 'jpg', 'bgr8')
                compressed_msg.data = np.array(
                    cv2.imencode('.jpg', frame)[1]).tostring()
                compressed_msg.header.frame_id = self.frame_id
                compressed_msg.header.stamp = now
                self.pub_compressed.publish(compressed_msg)

        self.client.subscribe(self.topic)
        self.client.on_message = on_message
        return


if __name__ == '__main__':
    rospy.init_node('camera_image_publisher')
    ImagePublisher()
    rospy.spin()
