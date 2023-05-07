#!/usr/bin/env python

import argparse
import logging
import sys
import time

import cv2

from aibo_selenium_ros import AIBOBrowserInterface

if __name__ == '__main__':

  logging.basicConfig(level=logging.INFO)
  logger = logging.getLogger(__name__)

  parser = argparse.ArgumentParser()

  parser.add_argument('webdriver')
  parser.add_argument('--id', default='')
  parser.add_argument('--password', default='')

  parser.add_argument('--manual-login', action='store_true')
  parser.add_argument('--headless', action='store_true')

  args = parser.parse_args()

  if not args.manual_login:
    interface = AIBOBrowserInterface(executable_path=args.webdriver,
                                     login_id=args.id,
                                     login_pw=args.password,
                                     auto_login=True,
                                     headless=args.headless)
  else:
    interface = AIBOBrowserInterface(executable_path=args.webdriver,
                                     login_id=args.id,
                                     login_pw=args.password,
                                     auto_login=False,
                                     headless=args.headless)
    input('Press Enter when logging if completed.')
    interface.initialized = True

  if not interface.initialized:
    logger.error('Initialization failed.')
    sys.exit(1)

  interface.start_watching()
  while True:
    image_rgb = interface.get_current_image()
    if image_rgb is None:
      logger.info('Restarting watching....')
      time.sleep(3.)
      ret = interface.continue_watching()
      logger.info('Restarted watching.')
    else:
      image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
      cv2.imshow('hoge', image_bgr)
      if cv2.waitKey(1) != -1:
        break

  cv2.destroyAllWindows()
