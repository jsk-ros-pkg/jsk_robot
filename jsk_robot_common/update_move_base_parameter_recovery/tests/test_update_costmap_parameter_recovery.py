#!/usr/bin/env python

from costmap_2d.cfg import Costmap2DConfig
from dynamic_reconfigure.server import Server
import rospy
import threading
import sys
import unittest
import rostest
PKG = 'update_move_base_parameter_recovery'
NAME = 'update_costmap_parameter_recovery_test'


class TestUpdateCostmapParameterRecovery(unittest.TestCase):

    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node(NAME)

        self.lock_for_config = threading.Lock()
        self.config = {}

        self.drs = Server(Costmap2DConfig, self.callback)

    def callback(self, config, level):
        with self.lock_for_config:
            self.config = config
        rospy.logwarn('Update parameters.')
        return config

    def test_update_costmap_parameter_recovery(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            with self.lock_for_config:
                if 'footprint' in self.config and \
                        self.config['footprint'] != '[]':
                    break
            rate.sleep()

        with self.lock_for_config:
            rospy.logwarn('result config: {}'.format(self.config))
            self.assertEqual(self.config['footprint'], '[[1.0,1.0]]')
            self.assertEqual(self.config['footprint_padding'], 1.0)
            self.assertEqual(self.config['robot_radius'], 1.0)

        rospy.logwarn('finished.')


if __name__ == '__main__':
    rostest.rosrun(
        PKG,
        NAME,
        TestUpdateCostmapParameterRecovery,
        sys.argv
    )
