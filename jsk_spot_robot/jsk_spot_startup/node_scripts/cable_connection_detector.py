#!/usr/bin/env python

import rospy
import spot_msgs.msg
import sensor_msgs.msg
from std_msgs.msg import Bool


class CalbeConnectionDetector:

    def __init__(self):

        self.pub = rospy.Publisher(
            '/spot/status/cable_connected', Bool, queue_size=1)

        self.msg_battery_spot = None
        self.msg_battery_laptop = None

        self.sub_battery_spot = rospy.Subscriber(
            '/spot/status/power_state',
            spot_msgs.msg.PowerState,
            self.callback_battery_spot)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.msg_battery_spot == None and self.msg_battery_laptop == None:
                continue
            self.pub.publish(Bool(self.check_connection()))

    def check_connection(self):

        if self.msg_battery_spot is not None and\
                self.msg_battery_spot.shore_power_state == spot_msgs.msg.PowerState.STATE_ON_SHORE_POWER:
            return True
        else:
            return False

    def callback_battery_spot(self, msg):

        self.msg_battery_spot = msg



def main():

    rospy.init_node('cable_connection_detector')
    node = CalbeConnectionDetector()
    rospy.spin()


if __name__ == '__main__':
    main()