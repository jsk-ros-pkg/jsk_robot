#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys,subprocess,traceback
import rospy
import time
import os
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal
import actionlib
import roslib

class HokuyoScanChecker:
    def __init__(self):
        rospy.init_node('hokuyo_scan_checker_node', anonymous=True)
        self.no_topic_flag = False ## True if topic is currently not coming in.
        self.once_topic_flag = False ## True if the topic has been received at least once since the start of node execution.

        self.launch_process = None
        self.hokuyo_name = rospy.get_param('~hokuyo_name', 'base')

        # Create an Action client for the sound_play node
        self.sound_client = actionlib.SimpleActionClient('/robotsound', SoundRequestAction)
        self.sound_client.wait_for_server()

        # Threshold (in seconds) when a topic is not updated for a certain period of time
        self.timeout_threshold = 10.0

        # Time the topic was last updated
        self.last_image_time = time.time()

        self.say_something("{} scan check start".format(self.hokuyo_name))

        # Subscribing topic
        self.topic_sub = rospy.Subscriber('/{}_scan'.format(self.hokuyo_name), LaserScan, self.topic_callback)

    def topic_callback(self, msg):
        # Callback to be called when topic is updated
        self.last_image_time = time.time()
        if self.no_topic_flag or self.once_topic_flag==False:
            self.no_topic_flag = False
            self.once_topic_flag = True
            self.say_something("{} scan topic is arrive.".format(self.hokuyo_name))

    def check_timeout(self):
        # Check if the topic has not been updated for a certain period of time
        if time.time() - self.last_image_time > self.timeout_threshold:
            if self.no_topic_flag:
                return
            else:
                self.no_topic_flag = True
                self.say_something("I haven't seen the {} scan topic for {} seconds.".format(self.hokuyo_name, self.timeout_threshold))
                self.restart_hokuyo()

    def say_something(self, text):
        # Let the robot talk
        rospy.loginfo(text)

        # Create a SoundRequestGoal message
        sound_goal = SoundRequestGoal()
        sound_goal.sound_request.sound = SoundRequest.SAY
        sound_goal.sound_request.command = SoundRequest.PLAY_ONCE
        sound_goal.sound_request.volume = 1.0
        sound_goal.sound_request.arg = text

        # Send the SoundRequestGoal to the sound_play node
        self.sound_client.send_goal(sound_goal)

        # Wait for the result (you can add timeout if needed)
        self.sound_client.wait_for_result()

    def restart_hokuyo(self):
        rospy.logerr("Restarting {} hokuyo".format(self.hokuyo_name))
        retcode = -1
        if self.launch_process: ## Force termination if launch process exists
            self.launch_process.terminate()
            self.launch_process.wait()
        try:
            # 1. kill hokuyo node
            retcode = subprocess.call('rosnode kill /{}_hokuyo_node'.format(self.hokuyo_name), shell=True)
            retcode = subprocess.call('pkill -f {}_hokuyo_node'.format(self.hokuyo_name), shell=True)
            rospy.loginfo("Killed {} hokuyo node".format(self.hokuyo_name))
            time.sleep(10)

            # 2. reset hokuyo
            package_path = roslib.packages.get_pkg_dir('jsk_pr2_startup')
            script_path = os.path.join(package_path, 'jsk_pr2_sensors/hokuyo_reset_scripts')
            retcode = subprocess.call('{}/upgrade /etc/ros/sensors/{}_hokuyo {}/reset.cmd'.format(script_path, self.hokuyo_name, script_path), shell=True)
            self.say_something("Reset {} hokuyo".format(self.hokuyo_name))
            time.sleep(10)

            # 3 Restarting hokuyo node
            os.environ['ROS_ENV_LOADER'] = '/home/applications/ros/noetic/devel/env.sh'
            self.launch_process = subprocess.Popen(['roslaunch', 'jsk_pr2_startup', '{}_hokuyo.launch'.format(self.hokuyo_name)], env=os.environ)
            rospy.loginfo("Restart {} hokuyo node".format(self.hokuyo_name))
            time.sleep(30)
            rospy.loginfo("Restarting {} hokuyo node is Done".format(self.hokuyo_name))

        except Exception as e:
            rospy.logerr('[%s] Unable to kill %s hokuyo node, caught exception:\n%s', self.__class__.__name__, self.hokuyo_name, traceback.format_exc())

    def run(self):
        rate = rospy.Rate(1)  # Router plate: 1 Hz
        while not rospy.is_shutdown():
            self.check_timeout()
            rate.sleep()

if __name__ == '__main__':
    try:
        topic_checker = HokuyoScanChecker()
        topic_checker.run()
    except rospy.ROSInterruptException:
        pass
