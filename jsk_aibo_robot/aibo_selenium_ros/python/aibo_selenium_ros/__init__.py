import base64
import io
import logging
import time

import selenium
from imageio import imread
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.by import By

logger = logging.getLogger(__name__)


class AIBOBrowserInterface(object):

  def __init__(self,
               executable_path='',
               login_page_url='https://myaibo.aibo.sony.jp/#/',
               login_id='',
               login_pw='',
               auto_login=True,
               headless=False,
               timeout=20.):

    self.initialized = False

    options = Options()
    options.add_argument('--disable-blink-features=AutomationControlled')
    if headless:
      options.add_argument('--headless')

    self.driver = webdriver.Chrome(executable_path=executable_path,
                                   options=options)

    if not auto_login:
      self.driver.get(login_page_url)
      logger.info('Opened default page')
      return

    # Open login page
    self.driver.get(login_page_url)
    logger.info('Opened default page')

    # Press いますぐサインイン
    self.driver.find_elements(By.TAG_NAME, value='button')[0].click()
    logger.info('Opened login page')

    # Input login ID and enter
    deadline = time.perf_counter() + timeout
    success = False
    while time.perf_counter() < deadline:
      try:
        e_input = self.driver.find_element(By.ID, value='input-sign-in-id')
        e_button = self.driver.find_element(By.ID, value='button-sign-in')
        success = True
        break
      except selenium.common.exceptions.NoSuchElementException:
        continue
    if not success:
      logger.error('Failed to continue login process. Please proceed manually.')
      return
    else:
      time.sleep(1.)
      e_input.send_keys(login_id)
      e_button.click()
      logger.info('Input ID and clicked button')

    # Input PW and login
    deadline = time.perf_counter() + timeout
    success = False
    while time.perf_counter() < deadline:
      try:
        e_input = self.driver.find_element(
            By.ID, value='signin-password-input-password')
        e_button = self.driver.find_element(By.ID,
                                            value='signin-password-button')
        success = True
        break
      except selenium.common.exceptions.NoSuchElementException:
        continue
    if not success:
      logger.error('Failed to continue login process. Please proceed manually.')
      return
    else:
      time.sleep(1.)
      e_input.send_keys(login_pw)
      e_button.click()
      logger.info('Input PW and clicked button')

    time.sleep(10.)
    deadline = time.perf_counter() + timeout
    success = False
    while time.perf_counter() < deadline:
      try:
        for b in self.driver.find_elements(By.TAG_NAME, value='button'):
          if b.accessible_name == '今なに見てる？':
            success = True
            break
      except selenium.common.exceptions.NoSuchElementException:
        continue
    if success:
      logger.info('Initialized')
      self.initialized = True
    else:
      logger.error('Failed to continue login process. Please proceed manually.')

  def start_watching(self, timeout=5.):

    for b in self.driver.find_elements(By.TAG_NAME, value='button'):
      if b.accessible_name == '今なに見てる？' and b.is_displayed():
        b.click()
        break
    time.sleep(10.)

    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
      try:
        self.driver.find_element(By.TAG_NAME, value='video')
        return True
      except selenium.common.exceptions.NoSuchElementException:
        continue
    return False

  def continue_watching(self, timeout=5.):

    for b in self.driver.find_elements(By.TAG_NAME, value='button'):
      if b.accessible_name == 'OK':
        b.click()
        break
    time.sleep(3.)
    for b in self.driver.find_elements(By.TAG_NAME, value='button'):
      if b.accessible_name == '閉じる':
        b.click()
        break
    time.sleep(3.)
    for b in self.driver.find_elements(By.TAG_NAME, value='button'):
      if b.accessible_name == '今なに見てる？' and b.is_displayed():
        b.click()
        break
    time.sleep(10.)

    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
      try:
        self.driver.find_element(By.TAG_NAME, value='video')
        return True
      except selenium.common.exceptions.NoSuchElementException:
        continue
    return False

  def get_current_image(self):

    try:
      raw_data_image = self.driver.find_element(
          By.TAG_NAME, value='video').screenshot_as_base64
      cv_image_array_rgb = imread(io.BytesIO(base64.b64decode(raw_data_image)))
      return cv_image_array_rgb
    except selenium.common.exceptions.NoSuchElementException:
      return None

  def get_error_message(self):

    try:
      self.driver.find_element(By.CLASS_NAME, value='jss755').text
    except selenium.common.exceptions.NoSuchElementException:
      return None
