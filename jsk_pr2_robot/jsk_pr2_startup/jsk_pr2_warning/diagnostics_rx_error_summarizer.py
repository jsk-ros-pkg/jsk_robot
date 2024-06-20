#!/usr/bin/env python

import csv
import rospy
import argparse
from diagnostic_msgs.msg import DiagnosticArray


class DiagnosticsRxErrorSummarizer():
    def __init__(self, not_csv=False, file_name='diagnostic_agg_rx_error.csv', show_all_ports=False):
        self.subscribe()
        self.keys = ['timestamp']
        self.count = 0
        self.output_file = file_name
        self.not_csv = not_csv
        self.show_all_ports = show_all_ports

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
        timestamp = msg.header.stamp.secs
        self.values = [timestamp]
        print(self.count)
        flag = False
        thre_val = 0
        s_name_list = []
        key_list = []
        value_list = []
        for s in status:
            if 'values' not in dir(s):
                continue
            for v in s.values:
                if 'RX Error' in v.key:
                    if self.count == 0:
                        self.keys.append(s.name + ' - ' + v.key)
                    self.values.append(v.value)
                    if not ('Forwarded' in v.key): ### stdout does not include Forwarded
                        if self.show_all_ports:
                            print(str(timestamp) + " : " + s.name + " " + v.key + ": {}".format(v.value))
                        if not ('Master' in s.name):
                            flag = True
                            if int(v.value) > thre_val:
                                s_name_list.append(s.name)
                                value_list.append(v.value)
                                key_list.append(v.key)

        if not self.not_csv:
            if self.count == 0:
                self.write_keys()
            self.write_values()

        if flag:
            if len(s_name_list) > 0:
                print("[Devices with errors]")
                for s_name, key, value in zip(s_name_list, key_list, value_list):
                    print(str(timestamp) + " : " + s_name + " " + key + ": {}".format(value))
            print("--------------------------------------------")

        self.count += 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--unsave', action='store_true', help='do not save data in csv')
    parser.add_argument('--show_all_ports', action='store_true', help='show all ports result in stdout')
    parser.add_argument('--file_name', '-f', default='diagnostic_agg_rx_error.csv', help='name of the csv file to save')
    args = parser.parse_args()
    not_csv = args.unsave
    show_all_ports = args.show_all_ports
    file_name = args.file_name

    rospy.init_node('diagnostics_rx_errror_summarizer', anonymous=True)
    diagnostics_rx_errror_summarizer = DiagnosticsRxErrorSummarizer(not_csv=not_csv, file_name=file_name, show_all_ports=show_all_ports)
    rospy.spin()
