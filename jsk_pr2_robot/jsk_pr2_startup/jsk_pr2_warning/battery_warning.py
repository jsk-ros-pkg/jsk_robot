#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import division
from __future__ import unicode_literals

import actionlib
from collections import defaultdict
import os.path as osp
import math
import rospy
import pandas as pd

from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequest, SoundRequestGoal
from pr2_msgs.msg import PowerState
from diagnostic_msgs.msg import DiagnosticArray


class BatteryWarning(object):
    def __init__(self):
        # speak
        self.speak_client = actionlib.SimpleActionClient(
            "/robotsound_jp", SoundRequestAction)
        ok = self.speak_client.wait_for_server(rospy.Duration(10))
        if not ok:
            rospy.logfatal("Action /robotsound_jp is not advertised")
            exit(1)

        # param
        self.monitor_rate = rospy.get_param("~monitor_rate", 4)
        self.warning_temp = rospy.get_param("~warning_temperature", 42.0)
        self.min_capacity = rospy.get_param("~min_capacity", 800)
        self.warning_voltage = rospy.get_param("~warning_voltage", 14.0)
        self.critical_voltage = rospy.get_param("~critical_voltage", 13.7)
        self.warn_repeat_rate = rospy.get_param("~warn_repeat_rate", 180)
        self.log_rate = rospy.get_param("~log_rate", 10)
        self.log_path = rospy.get_param("~log_path", None)

        self.prev_plugged_in = None
        self.latest_status = None
        self.speech_history = defaultdict(lambda: rospy.Time(0))

        self.diag_sub = rospy.Subscriber(
            "/diagnostics_agg", DiagnosticArray, self.diag_cb, queue_size=1)
        self.stat_timer = rospy.Timer(
            rospy.Duration(self.monitor_rate), self.stat_cb)
        if self.log_path is not None:
            self.log_timer = rospy.Timer(
                rospy.Duration(self.log_rate), self.log_cb)

    def speak(self, sentence):
        # Pick first 4 characters as a keyword instead of using whole sentence
        # because sentence can have variables like 100%, 90%, etc.
        key = sentence[:4]
        if self.speech_history[key] + rospy.Duration(self.warn_repeat_rate) > rospy.Time.now():
            return
        self.speech_history[key] = rospy.Time.now()
        req = SoundRequest()
        req.command = SoundRequest.PLAY_ONCE
        req.sound = SoundRequest.SAY
        req.arg = sentence
        req.arg2 = "ja"
        req.volume = 1.0
        self.speak_client.send_goal(SoundRequestGoal(sound_request=req))
        self.speak_client.wait_for_result(timeout=rospy.Duration(10))

    def log_cb(self, event):
        try:
            if osp.exists(self.log_path):
                orig_df = pd.read_pickle(self.log_path)
                df = orig_df.append(self.latest_status, ignore_index=True)
            else:
                df = self.latest_status
            df.to_pickle(self.log_path)
        except Exception as e:
            if osp.exists(self.log_path):
                try:
                    import shutil
                    shutil.move(self.log_path, self.log_path + ".bak")
                    rospy.logwarn("Moved old file to %s" % (self.log_path + ".bak"))
                except:
                    pass

    def stat_cb(self, event):
        df = self.latest_status

        if self.latest_status is None:
            return

        try:
            max_temp = df["Temperature (C)"].astype(float).max()
            rospy.logdebug("temperature: %s" % max_temp)
            if 60 > max_temp > self.warning_temp:
                self.speak("バッテリ温度%.1f度。暑いです。部屋の温度を下げてください。" % max_temp)
        except KeyError:
            pass
        except ValueError:
            pass

        try:
            plugged_in = df["Power Present"].eq("True").any()
            if self.prev_plugged_in is None:
                self.prev_plugged_in = not plugged_in
            prev_plugged_in, self.prev_plugged_in = self.prev_plugged_in, plugged_in
            if plugged_in:
                if not prev_plugged_in:
                    attf_max = df["Average Time To Full (min)"].astype(int).max()
                    rospy.loginfo("Average Time to full: %s" % attf_max)
                    if attf_max > 0:
                        self.speak("フル充電まで%s分です。" % attf_max)
                return
        except KeyError:
            pass
        except ValueError:
            pass

        try:
            rc = df["Remaining Capacity (mAh)"].astype(float).sub(self.min_capacity)
            fc = df["Full Charge Capacity (mAh)"].dropna(
                how='all').astype(int).sub(self.min_capacity)
            min_perc = int(rc.where(rc > 0).div(fc).min() * 100.0)

            if (prev_plugged_in and plugged_in) or min_perc < 50:
                self.speak("電池残り%s％です。" % min_perc)
            if 15 < min_perc < 30:
                self.speak("充電してください。")
            elif 0 <= min_perc < 15:
                self.speak("もう限界です！")
        except KeyError:
            pass
        except ValueError:
            pass

        try:
            voltage = df["Voltage (mV)"].astype(int).min() / 1000
            if (prev_plugged_in and plugged_in) or \
               voltage < self.warning_voltage:
                self.speak("電池電圧%.1fボルトです。" % voltage)
            if self.critical_voltage < voltage < self.warning_voltage:
                self.speak("充電してください。")
            elif voltage <= self.critical_voltage:
                self.speak("もう限界です！")
        except KeyError:
            pass
        except ValueError:
            pass

    def diag_cb(self, msg):
        stamp = msg.header.stamp.secs
        batt_status = filter(
            lambda s: s.name.startswith("/Power System/Smart Battery"),
            msg.status)
        status = []
        for s in sorted(batt_status, key=lambda s: s.name):
            stat = {d.key: d.value for d in s.values}
            stat["Name"] = s.name
            stat["Time Stamp"] = stamp
            status.append(stat)
        df = pd.DataFrame(status)
        if self.latest_status is not None:
            self.prev_status = self.latest_status
        self.latest_status = df

if __name__ == '__main__':
    rospy.init_node("battery_warning")
    b = BatteryWarning()
    rospy.spin()
