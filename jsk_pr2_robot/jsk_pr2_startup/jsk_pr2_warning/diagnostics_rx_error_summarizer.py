#!/usr/bin/env python

import csv
import rospy
import argparse
from diagnostic_msgs.msg import DiagnosticArray


class DiagnosticsRxErrorSummarizer():
    def __init__(self, is_csv=False):
        self.subscribe()
        self.keys = ['timestamp']
        self.count = 0
        self.output_file = 'diagnostic_agg_rx_error.csv'
        self.is_csv = is_csv

    def subscribe(self):
        self.sub = rospy.Subscriber("/diagnostics_agg",
                                    DiagnosticArray,
                                    self.callback,
                                    queue_size=1)

    def write_keys(self):
        with open(self.output_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(self.keys)

    def write_values(self):
        with open(self.output_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(self.values)

    def callback(self, msg):
        status = msg.status
        self.values = [msg.header.stamp.secs]
        if self.is_csv:
            print(self.count)
            for s in status:
                if 'values' not in dir(s):
                    continue
                for v in s.values:
                    if 'RX Error' in v.key:
                        if self.count == 0:
                            self.keys.append(s.name + ' - ' + v.key)
                            self.values.append(v.value)

            if self.count == 0:
                self.write_keys()
            self.write_values()

            self.count += 1
        else:
            flag = False
            max_val = 0
            max_key = None
            max_s_name = None
            for s in status:
                if 'values' not in dir(s):
                    continue
                for v in s.values:
                    if 'RX Error' in v.key and not ('Forwarded' in v.key):
                        print(s.name + " " + v.key + ": {}".format(v.value))
                        if not ('Master' in s.name):
                            flag = True
                            if int(v.value) > max_val:
                                max_val = int(v.value)
                                max_key = [v.key]
                                max_s_name = [s.name]
                            elif int(v.value) == max_val and max_val != 0:
                                max_key.append(v.key)
                                max_s_name.append(s.name)

            if flag:
                if max_val > 0:
                    print("[Max Device]")
                    for s_name, key in zip(max_s_name, max_key):
                        print(s_name + " " + key + ": {}".format(max_val))
                print("--------------------------------------------")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', action='store_true', help='save data in csv or not')
    args = parser.parse_args()
    is_csv= args.csv

    rospy.init_node('diagnostics_rx_errror_summarizer', anonymous=True)
    diagnostics_rx_errror_summarizer = DiagnosticsRxErrorSummarizer(is_csv=is_csv)
    rospy.spin()
