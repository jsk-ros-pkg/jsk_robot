#!/usr/bin/env python

import rospy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


class LaptopBatteryVisualizer(object):

    def __init__(self):

        rospy.init_node('spot_battery_visualizer')

        self._sub = rospy.Subscriber('/laptop_charge', BatteryState, self._cb)
        self._pub = rospy.Publisher(
            '/spot/status/laptop_battery_percentage', Float32, queue_size=1)

    def _cb(self, msg):

        pub_msg = Float32()
        pub_msg.data = msg.percentage
        self._pub.publish(pub_msg)


def main():

    battery_visualizer = LaptopBatteryVisualizer()
    rospy.spin()


if __name__ == '__main__':
    main()
