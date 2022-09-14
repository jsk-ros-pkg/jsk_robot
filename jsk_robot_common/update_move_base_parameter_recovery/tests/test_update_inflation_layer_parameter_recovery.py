#!/usr/bin/env python

from costmap_2d.cfg import InflationPluginConfig
from dynamic_reconfigure.server import Server
import rospy
import threading
import sys
import unittest
import rostest
PKG = 'update_move_base_parameter_recovery'
NAME = 'update_inflation_layer_parameter_recovery_test'


class TestUpdateInflationPluginParameterRecovery(unittest.TestCase):

    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node(NAME)

        self.lock_for_config = threading.Lock()
        self.config = {}

        self.drs = Server(InflationPluginConfig, self.callback)

    def callback(self, config, level):
        with self.lock_for_config:
            self.config = config
        rospy.logwarn('Update parameters.')
        return config

    def test_update_inflation_layer_parameter_recovery(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            with self.lock_for_config:
                if 'inflate_unknown' in self.config and \
                        self.config['inflate_unknown']:
                    break
            rate.sleep()

        with self.lock_for_config:
            self.assertEqual(self.config['cost_scaling_factor'], 1.0)
            self.assertEqual(self.config['enabled'], False)
            self.assertEqual(self.config['inflate_unknown'], True)
            self.assertEqual(self.config['inflation_radius'], 1.0)

        rospy.logwarn('finished.')


if __name__ == '__main__':
    rostest.rosrun(
        PKG,
        NAME,
        TestUpdateInflationPluginParameterRecovery,
        sys.argv
    )
