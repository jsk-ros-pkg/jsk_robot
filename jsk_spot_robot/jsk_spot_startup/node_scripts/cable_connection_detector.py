#!/usr/bin/env python

import rospy
from spot_msgs.msg import BatteryStateArray, BatteryState
from std_msgs.msg import Bool

class CalbeConnectionDetector:

    def __init__(self):

        rospy.pub = rospy.Publisher('/spot/status/cable_connected',Bool,queue_size=1)
        rospy.sub_battery_state = rospy.Subscriber('/spot/status/battery_states',BatteryStateArray,self.callback_battery_state)

    def callback_battery_state(self,msg):

        if msg.battery_states[0].status == BatteryState.STATUS_DISCHARGING:
            rospy.pub.publish(Bool(False))
        else:
            rospy.pub.publish(Bool(True))

def main():

    rospy.init_node('cable_connection_detector')
    node = CalbeConnectionDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
