#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Takayuki Murooka <t-murooka@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import division
from __future__ import unicode_literals

import actionlib
from collections import OrderedDict
from collections import defaultdict
import os.path as osp
import rospy

from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequest, SoundRequestGoal

# TODO safe_rateとmonitor_rateの同期

class DeviceMonitorMethod(object):
    def __init__(self, monitor_method, monitor_rate=180):
        self.latest_exist_time = rospy.Time.now()

        if monitor_method[1] == "topic":
            topic_name = monitor_method[0]
            self.sub = rospy.Subscriber(topic_name, rospy.AnyMsg, self.cb, queue_size=1)
        elif monitor_method[1] == "file":
            pass
        elif monitor_method[1] == "node":
            pass

    def cb(self, msg):
        self.latest_sub_time = rospy.Time.now()

    def monitor(self):
        if monitor_method[1] == "topic":
            pass
        elif monitor_method[1] == "file":
            if osp.exists(monitor_method[0]):
                self.latest_exist_time = rospy.Time.now()
        elif monitor_method[1] == "node":
            pass

        is_exist = rospy.Time.now() - self.latest_exist_time < safe_rate
        if is_exist:
            return "{}が見えません。"
        else:
            return False

class DeviceInfo(object):
    def __init__(self, name, pronunciation, monitor_methods, monitor_rate=180):
        self.name = name
        self.pronumciation = pronunciation
        self.monitor_methods = []
        for i in range(len(monitor_methods)):
            self.monitor_methods.append(DeviceMonitorMethod(monitor_methods[i], monitor_rate))

        self.is_exist = True

        self.speech_history = defaultdict(lambda: rospy.Time(0))

    def monitor(self):
        for monitor_method in self.monitor_methods:
            speech = monitor_method.monitor():
            if speech:
                return check_speech.format(pronunciation)

    def check_speech(self, sentence):
        # Pick first 4 characters as a keyword instead of using whole sentence
        # because sentence can have variables like 100%, 90%, etc.
        key = sentence[:4]
        if self.speech_history[key] + rospy.Duration(self.monitor_rate) > rospy.Time.now():
            return
        self.speech_history[key] = rospy.Time.now()
        return sentence

class DeviceMonitor(object):
    def __init__(self):
        # speak
        self.speak_client = actionlib.SimpleActionClient(
            "/robotsound_jp", SoundRequestAction)
        ok = self.speak_client.wait_for_server(rospy.Duration(10))
        if not ok:
            rospy.logfatal("Action /robotsound_jp is not advertised")
            exit(1)

            # devices
        self.device_info_list = [DeviceInfo("base_laser", "ベースレーザ", [["/dev/ttyACM1", "file"], ["/base_scan", "topic"]], self.monitor_rate)]

        # self.device_info_list = [DeviceInfo("base_laser", "ベースレーザ", [["/base_scan", "topic"], ["/base_hokuyo_node", "node"], ["/etc/ros/sensors/base_hokuyo", "file"], ["/dev/sensors/hokuyo_H1002732", "file"], ["/dev/ttyACM1", "file"]]),
        #                          DeviceInfo("tilt_laser", "チルトレーザ", [["/tilt_scan", "topic"], ["/tilt_hokuyo_node", "node"], ["/etc/ros/sensors/tilt_hokuyo", "file"], ["/dev/sensors/hokuyo_H0902645", "file"], ["/dev/ttyACM0", "file"]]),
        #                          DeviceInfo("kinect_rgb", "キネクトアールジービー", [["/kinect_head/rgb/image_rect_color", "topic"], ["/dev/bus/usb/001/007", "file"]]),
        #                          DeviceInfo("kinect_depth", "キネクトデプス", [["/kinect_head/depth_registered/points", "topic"], ["/dev/hoge", "file"]])]

        self.monitor_timer = rospy.Timer(
            rospy.Duration(self.monitor_rate), self.monitor_cb)

    def monitor_cb(self, timer):
        for device_info in self.device_info_list:
            self.speak(device_info.monitor())

    def speak(self, sentence):
        req = SoundRequest()
        req.command = SoundRequest.PLAY_ONCE
        req.sound = SoundRequest.SAY
        req.arg = sentence
        req.arg2 = "ja"
        req.volume = 1.0
        self.speak_client.send_goal(SoundRequestGoal(sound_request=req))
        self.speak_client.wait_for_result(timeout=rospy.Duration(10))

if __name__ == '__main__':
    rospy.init_node("device_monitor")
    d = DeviceMonitor()
    rospy.spin()
