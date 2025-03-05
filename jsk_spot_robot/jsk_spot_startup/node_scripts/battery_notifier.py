#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerRequest
from spot_msgs.msg import BatteryStateArray
from sensor_msgs.msg import BatteryState

from sound_play.libsoundplay import SoundClient


class SpotBatteryNotifier(object):

    def __init__(self):

        self._battery_spot = None
        self._battery_temperature = 0
        self._is_connected = False
        self.last_warn_bat_temp_time = rospy.get_time()

        self._sub_spot = rospy.Subscriber(
            '/spot/status/battery_states',
            BatteryStateArray,
            self._cb_spot)
        self._sub_connected = rospy.Subscriber(
            '/spot/status/cable_connected',
            Bool,
            self._cb_connected)

        sound_client = SoundClient(
            blocking=False,
            sound_action='/robotsound_jp',
            sound_topic='/robotsound_jp'
        )

        threshold_warning_battery_temperature =\
            float(rospy.get_param('~threshold_warning_battery_temperature', 45))

        threshold_warning_spot = float(
            rospy.get_param('~threshold_warning_spot', 20))

        threshold_return_spot = float(
            rospy.get_param('~threshold_return_spot', 15))

        threshold_estop_spot = float(
            rospy.get_param('~threshold_estop_spot', 5))

        notification_duration = float(
            rospy.get_param('~notification_duration', 300))

        rate = rospy.Rate(0.1)
        last_notified = rospy.Time.now()
        while not rospy.is_shutdown():

            rate.sleep()

            if not self._is_connected:

                if (rospy.Time.now() - last_notified).to_sec() > notification_duration:
                    sound_client.say('バッテリー残量、{}、パーセントです。'.format(
                        int(self._battery_spot)), volume=0.3)
                    last_notified = rospy.Time.now()

                if (self._battery_spot is not None and self._battery_spot < threshold_estop_spot):
                    rospy.logerr('Battery is low. Estop.')
                    sound_client.say('バッテリー残量が少ないため、動作を停止します')
                    rospy.ServiceProxy('/spot/estop/gentle', Trigger)(TriggerRequest())
                    rospy.ServiceProxy('/spot/estop/hard', Trigger)(TriggerRequest())

                elif (self._battery_spot is not None and self._battery_spot < threshold_return_spot):
                    rospy.logerr('Battery is low. Returning to home.')
                    sound_client.say('バッテリー残量が少ないため、ドックに戻ります')
                    self._call_go_back_home()

                elif (self._battery_spot is not None and self._battery_spot < threshold_warning_spot):
                    rospy.logwarn('Battery is low. Spot: {}'.format(self._battery_spot))
                    sound_client.say('バッテリー残量が少ないです。本体が、{}、パーセントです。'.format(
                        int(self._battery_spot)))

            if self._battery_temperature > threshold_warning_battery_temperature\
                    and (rospy.get_time() - self.last_warn_bat_temp_time) > 180:
                rospy.logerr('Battery temperature is high. Battery temp:{:.2f} > threshold:{}'
                             .format(self._battery_temperature, threshold_warning_battery_temperature))
                sound_client.say('バッテリー温度が高いです。')
                self.last_warn_bat_temp_time = rospy.get_time()

    def _cb_spot(self, msg):

        self._battery_spot = msg.battery_states[0].charge_percentage
        self._battery_temperature = max(msg.battery_states[0].temperatures)

    def _cb_connected(self, msg):

        self._is_connected = msg.data

    def _call_estop_gentle(self):

        try:
            trigger = rospy.ServiceProxy('/spot/estop/gentle', Trigger)
            ret = trigger()
            rospy.loginfo('Call %s and received %s(%s)'%(trigger.resolved_name, ret.sucess, ret.message))
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s'%e)

    def _call_estop_hard(self):

        try:
            trigger = rospy.ServiceProxy('/spot/estop/hard', Trigger)
            ret = trigger()
            rospy.loginfo('Call %s and received %s(%s)'%(trigger.resolved_name, ret.sucess, ret.message))
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s'%e)

    def _call_go_back_home(self):

        try:
            trigger = rospy.ServiceProxy('/spot/dock_fixed_id', Trigger)
            ret = trigger()
            rospy.loginfo('Call %s and received %s(%s)'%(trigger.resolved_name, ret.sucess, ret.message))
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s'%e)


def main():

    rospy.init_node('battery_notifier')
    battery_notifier = SpotBatteryNotifier()
    rospy.spin()


if __name__ == '__main__':
    main()
