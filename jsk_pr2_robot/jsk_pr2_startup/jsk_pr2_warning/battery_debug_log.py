#!/usr/bin/env python

import rospy
import commands
import os.path
from pr2_msgs.msg import BatteryServer

class BatteryLog:
    def __init__(self):
        rospy.Subscriber("/battery/server", BatteryServer, self.battery_server_cb)
        self.store_path = "/var/ros/battery_debug_log/battery_debug.log"
        self.sys_uptime = int(float(commands.getoutput("cat /proc/uptime |awk '{print $1}'")))
        self.date_in_sec = int(commands.getoutput("date +\"%s\""))
        self.sys_started_date = self.date_in_sec - self.sys_uptime
        self.log = ["\n","\n","\n","\n"]
        self.appending = True
        if not os.path.isfile(self.store_path):
            with open(self.store_path, "w") as f:
                f.write("system started date(s)|date|ID|Rel.(%)|Abs.(%)|Voltage(mV)|Temperture(C)|\n\n\n\n\n")
                f.close()
        if (self.sys_started_date - self.read_sys_started_date() <= 60 * 1): # 1mins, consider to be same boot
            self.appending =False


    def battery_server_cb(self, msg):
        bats = msg.battery
        charge_list = [bat.batReg[14] for bat in bats]
        min_abs_charge = reduce(min, charge_list)
        sub_id = charge_list.index(min_abs_charge)
        bat_id = (msg.id + sub_id / 10.0)
        rel_charge = bats[sub_id].batReg[13]
        voltage = bats[sub_id].batReg[9]
        temp = bats[sub_id].batReg[8] * 0.1 - 273.15
        
        self.log[msg.id] = "{0} {1} {2} {3} {4} {5} {6}\n".format(self.sys_started_date, commands.getoutput("date"), bat_id, rel_charge, min_abs_charge,  voltage, temp)
        rospy.loginfo(self.log[msg.id])
        if self.appending:
            self.append_data()
        else:
            self.replace_data()

    def read_sys_started_date(self):
        with open(self.store_path, 'r') as f:
            lst = f.readlines()
            try:
                return int(float(str.split(lst[-1])[0]))
            except Exception as e:
                rospy.logwarn("faild to read system started date from file: %s" % e)
                return self.sys_started_date

    def replace_data(self):
        f = open(self.store_path, 'r')
        lines = f.readlines()
        f.close()
        f= open(self.store_path, 'w')
        f.writelines([l for l in lines[:-4]])
        for s in self.log:
            f.write(s)
        f.close()

    def append_data(self):
        self.appending = False
        with open(self.store_path, 'a') as f:
            f.write("\nsystem started date(s)|date|ID|Rel.(%)|Abs.(%)|Voltage(mV)|Temperture(C)|\n")
            for s in self.log:
                f.write(s)



if __name__ == "__main__":
    rospy.init_node('battery_log')
    obj = BatteryLog()
    rospy.spin()
        
