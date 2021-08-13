#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from spot_msgs.msg import BatteryStateArray
from sensor_msgs.msg import BatteryState

from spot_ros_client.libspotros import SpotRosClient
from sound_play.libsoundplay import SoundClient


class SpotBatteryNotifier(object):

    def __init__(self):

        self._battery_spot = 0
        self._battery_laptop = 0

        self._sub_spot = rospy.Subscriber(
                                '/spot/status/battery_states',
                                BatteryStateArray,
                                self._cb_spot )
        self._sub_laptop = rospy.Subscriber(
                                '/laptop_charge',
                                BatteryState,
                                self._cb_laptop )

        spot_client = SpotRosClient()
        sound_client = SoundClient(
                        blocking=False,
                        sound_action='/robotsound_jp',
                        sound_topic='/robotsound_jp'
                        )

        threshold_warning_spot = float(rospy.get_param('~threshold_warning_spot', 20))
        threshold_warning_laptop = float(rospy.get_param('~threshold_warning_laptop', 20))

        threshold_estop_spot = float(rospy.get_param('~threshold_estop_spot', 5))
        threshold_estop_laptop = float(rospy.get_param('~threshold_estop_laptop', 5))

        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():

            rate.sleep()

            if self._battery_spot < threshold_warning_spot or\
                    self._battery_laptop < threshold_warning_laptop:
                rospy.logwarn('Battery is low. Spot: {}, Laptop: {}'.format(self._battery_spot,self._battery_laptop))
                sound_client.say('バッテリー残量が少ないです。')

            if self._battery_spot < threshold_estop_spot or\
                    self._battery_laptop < threshold_estop_laptop:
                rospy.logerr('Battery is low. Estop.')
                sound_client.say('バッテリー残量が少ないため、動作を停止します')
                spot_client.estop_gentle()
                spot_client.estop_hard()


    def _cb_spot(self, msg):

        self._battery_spot = msg.battery_states[0].charge_percentage

    def _cb_laptop(self, msg):

        self._battery_laptop = msg.percentage
