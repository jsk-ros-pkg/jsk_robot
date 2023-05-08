#!/usr/bin/env python

import logging
import sys
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from aibo_selenium_ros import AIBOBrowserInterface

if __name__ == '__main__':

  rospy.init_node('aibo_selenium_ros')

  pub = rospy.Publisher('~image', Image, queue_size=1)
  cv_bridge = CvBridge()

  logging.basicConfig(level=logging.INFO)
  logger = logging.getLogger(__name__)

  webdriver = rospy.get_param('~webdriver')
  login_id = rospy.get_param('~login_id', '')
  login_password = rospy.get_param('~login_password', '')

  auto_login = bool(rospy.get_param('~auto_login', False))
  headless = bool(rospy.get_param('~headless', False))

  interface = AIBOBrowserInterface(executable_path=webdriver,
                                   login_id=login_id,
                                   login_pw=login_password,
                                   auto_login=auto_login,
                                   headless=headless)
  if not auto_login:
    input('Press Enter when logging if completed.')
    interface.initialized = True

  if not interface.initialized:
    rospy.logerr('Initialization failed.')
    input('Press Enter when logging if completed.')

  interface.start_watching()
  while True:
    image_rgb = interface.get_current_image()
    if image_rgb is None:
      rospy.loginfo('Restarting watching....')
      time.sleep(3.)
      ret = interface.continue_watching()
      rospy.loginfo('Restarted watching.')
    else:
      msg = cv_bridge.cv2_to_imgmsg(cv2.cvtColor(image_rgb, cv2.COLOR_RGBA2RGB),
                                    encoding='rgb8')
      rospy.loginfo('Publish a message')
      pub.publish(msg)
