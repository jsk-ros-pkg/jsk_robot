#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from datetime import datetime
import csv
import json
import os
import requests
import rospy
from diagnostic_msgs.msg import DiagnosticArray
import traceback
import stat


def is_file_writable(path):
    if not os.path.exists(path):
        return True  # if file does not exist, any file is writable there
    st = os.stat(path)
    return (bool(st.st_mode & stat.S_IWUSR) and
            bool(st.st_mode & stat.S_IWGRP) and
            bool(st.st_mode & stat.S_IWOTH))


class BatteryLogger(object):
    def __init__(self, period=120):
        self.period = rospy.Duration(period)
        self.last_update = rospy.Time(0)

    def write(self, date, info):
        raise NotImplementedError()


class FileLogger(BatteryLogger):
    def __init__(self, out_dir):
        super(FileLogger, self).__init__(period=600)
        self.out_dir = out_dir
        if not os.path.isdir(self.out_dir):
            if os.path.exists(self.out_dir):
                raise RuntimeError(
                    "Output directory path already exitsts as file: %s" % self.out_dir)
            orig_umask = os.umask(0)
            try:
                os.makedirs(self.out_dir, 0777)
            finally:
                os.umask(orig_umask)

    def write(self, date, info):
        filename = os.path.join(self.out_dir,
                                datetime.now().strftime("battery_%Y-%m-%d.log"))
        lines = []
        index = ["HardwareID", "CycleCount", "FullCapacity", "RemainingCapacity",
                 "Voltage", "Current", "Temperature"]
        if not os.path.exists(filename):
            # write index
            lines.append(["Date", "Name"] + index)

        for name, batt in info.items():
            values = []
            for k in index:
                if k in batt:
                    values.append(str(batt[k]))
            lines.append([date.secs, name] + values)

        with open(filename, "a") as f:
            writer = csv.writer(f, lineterminator=os.linesep)
            writer.writerows(lines)
        if not is_file_writable(filename):
            orig_umask = os.umask(0)
            try:
                os.chmod(os.path.abspath(filename), 0777)
            finally:
                os.umask(orig_umask)


class DweetLogger(BatteryLogger):
    def __init__(self, prefix):
        super(DweetLogger, self).__init__(period=10)
        self.uid = "" + prefix + "-" + rospy.get_param("/robot/name")

    def write(self, date, info):
        info["date"] = date.secs
        res = requests.post("http://dweet.io/dweet/for/{uid}".format(uid=self.uid),
                            json=info)
        assert res.ok and json.loads(res.content)["this"] == "succeeded"


class BatteryInfoAggregator(object):
    def __init__(self):
        self.loggers = self.parse_params()
        if not self.loggers:
            rospy.logwarn("No logger initialized. Exiting...")
            rospy.signal_shutdown("shutdown")

        self.sub_diag = rospy.Subscriber(
            "/diagnostics_agg", DiagnosticArray, self.diag_callback)

    def parse_params(self):
        loggers = []
        for info in rospy.get_param("~loggers"):
            logger = None
            if info["type"] == "file":
                try:
                    out_dir = info["out_dir"]
                    loggers.append(FileLogger(out_dir))
                except Exception as e:
                    rospy.logerr("Failed to init logger type 'file': %s" % str(e))
            elif info["type"] == "dweet":
                try:
                    prefix = info["prefix"]
                    loggers.append(DweetLogger(prefix))
                except Exception as e:
                    rospy.logerr("Failed to init logger type 'dweet': %s" % str(e))
            else:
                rospy.logerr("Unsupported logger: %s" % info["type"])
                continue
        rospy.loginfo("%d loggers initialized" % len(loggers))
        return loggers

    def diag_callback(self, msg):
        # aggregate
        results = {}
        for s in msg.status:
            if s.name.startswith("/Power System/Smart Battery"):
                if s.name not in results:
                    results[s.name] = { "HardwareID": s.hardware_id }
                for kv in s.values:
                    if(kv.key.startswith("Full Charge Capacity (mAh)")):
                        results[s.name]["FullCapacity"] = int(kv.value)
                    elif kv.key.startswith("Remaining Capacity"):
                        results[s.name]["RemainingCapacity"] = int(kv.value)
                    elif kv.key.startswith("Batery Status"):
                        results[s.name]["Status"] = int(kv.value)
                    elif kv.key.startswith("Cycle Count"):
                        results[s.name]["CycleCount"] = int(kv.value)
                    elif kv.key.startswith("Manufacture Date"):
                        results[s.name]["ManufactureDate"] = kv.value
                    elif kv.key.startswith("Voltage (mV)"):
                        results[s.name]["Voltage"] = int(kv.value)
                    elif kv.key.startswith("Current (mA)"):
                        results[s.name]["Current"] = int(kv.value)
                    elif kv.key.startswith("Temperature (C)"):
                        results[s.name]["Temperature"] = float(kv.value)

        # log
        for logger in self.loggers:
            if logger.last_update + logger.period < msg.header.stamp:
                try:
                    logger.write(msg.header.stamp, results)
                    logger.last_update = msg.header.stamp
                except Exception as e:
                    rospy.logerr("[%s] Failed to write to logger: %s", type(logger).__name__, str(e))
                    rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    rospy.init_node("battery_logger")
    agg = BatteryInfoAggregator()
    rospy.spin()
