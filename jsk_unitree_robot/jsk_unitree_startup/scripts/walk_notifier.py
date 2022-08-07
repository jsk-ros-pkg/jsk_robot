#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import subprocess
import time
import re
import json
import tempfile

import PIL.Image
import sensor_msgs.msg
from std_srvs.srv import Trigger
import rospy
import cv_bridge


def send_mail(place, robot_name, sender_address, receiver_address, attachment=None):
    """
    Send mail with mailutils
    """
    mail_title = u"{}、お散歩中です。".format(robot_name)
    message = u"お散歩してます。\\n今は{}を歩いているよ。".format(place)
    cmd = u"echo -e '{}'".format(message)
    cmd += u" | /usr/bin/mail -s '{}' -r {} {}".format(
        mail_title, sender_address, receiver_address)
    if attachment is not None:
        cmd += ' -A {}'.format(attachment)
    rospy.logerr('Executing: {}'.format(cmd.encode('utf-8')))
    exit_code = subprocess.call(cmd.encode('utf-8'), shell=True)


class WalkNotifier(object):

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.robot_name = rospy.get_param('~robot_name').strip()
        self.sub = rospy.Subscriber('~input_image',
                                    sensor_msgs.msg.Image,
                                    callback=self.callback,
                                    queue_size=1)
        self.sender_address = rospy.get_param('~sender_address')
        self.receiver_address = rospy.get_param('~receiver_address')
        self.notify_interval = int(rospy.get_param('~notify_interval', 180))
        rospy.wait_for_service('~get_location')
        self.wait_image()

    def callback(self, img_msg):
        self.img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

    def wait_image(self):
        self.img = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.img is None:
            rate.sleep()
            rospy.loginfo('[WalkMail] waiting image')

    def get_place(self):
        response = rospy.ServiceProxy('~get_location', Trigger)()
        address = json.loads(response.message)
        a = address['results'][0]['formatted_address']
        print_address = " ".join(a.split(' ')[1:])

        if self.img is not None:
            _, img_path = tempfile.mkstemp(suffix='.jpg')
            PIL.Image.fromarray(self.img[..., ::-1]).save(img_path)
            send_mail(print_address, self.robot_name,
                      self.sender_address, self.receiver_address,
                      img_path)
            os.remove(img_path)
        else:
            send_mail(print_address, self.robot_name,
                      self.sender_address, self.receiver_address,
                      img_path)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            notifier.get_place()
            time.sleep(self.notify_interval)


if __name__ == '__main__':
    rospy.init_node('walk_notifier')
    notifier = WalkNotifier()
    notifier.run()
