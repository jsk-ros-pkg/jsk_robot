#!/usr/bin/env python

import rospy
from spot_msgs.msg import BatteryStateArray
from std_msgs.msg import Float32

class SpotBatteryVisualizer(object):

    def __init__(self):

        rospy.init_node( 'spot_battery_visualizer' )

        self._sub = rospy.Subscriber( '/spot/status/battery_states', BatteryStateArray, self._cb )
        self._pub = rospy.Publisher( '/spot/status/battery_percentage', Float32, queue_size=1 )

    def _cb(self, msg):

        pub_msg = Float32()
        pub_msg.data = msg.battery_states[0].charge_percentage
        self._pub.publish(pub_msg)

def main():

    battery_visualizer = SpotBatteryVisualizer()
    rospy.spin()

if __name__=='__main__':
    main()
