#!/usr/bin/env python
###############################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, Kei OKada
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
###############################################################################

import sys
import time
import unittest

import rospy

from rosgraph_msgs.msg import Log

PKG = 'jsk_robot_startup'
NAME = 'test_common_logger'

class RosoutTest(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node(NAME)
        self.timeout = rospy.get_param('~timeout', 30)

        self.base_trajectory_logger = False
        self.sub = rospy.Subscriber('rosout', Log, self._callback)

    def _callback(self, msg):
        if msg.name == "/lifelog/base_trajectory_logger":
            self.base_trajectory_logger = True

    def test_publish(self):
        t_start = time.time()
        while not rospy.is_shutdown():
            if time.time() - t_start > self.timeout:
                self.fail('Timed out');

            if self.base_trajectory_logger:
                assert True, "base_trajectory_logger is started"
                break
            rospy.sleep(0.01)

if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, RosoutTest, sys.argv)

