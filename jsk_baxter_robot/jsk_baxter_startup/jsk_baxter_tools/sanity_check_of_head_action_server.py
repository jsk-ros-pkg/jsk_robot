#!/usr/bin/env python

# Modified work: Copyright (c) 2017, JSK Lab.
# Original work of HeadClient:
# -----------------------------------------------------------------------------
# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# -----------------------------------------------------------------------------

import sys

import actionlib
from control_msgs.msg import SingleJointPositionAction
from control_msgs.msg import SingleJointPositionGoal
import rospy

import baxter_interface


class HeadClient(object):
    def __init__(self):
        ns = 'robot/head/head_action'
        self._client = actionlib.SimpleActionClient(
            ns,
            SingleJointPositionAction
        )
        self._goal = SingleJointPositionGoal()

        # Wait 10 Seconds for the head action server to start or exit
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - Head Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.clear()

    def command(self, position, velocity):
        self._goal.position = position
        self._goal.max_velocity = velocity
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=5.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def clear(self):
        self._goal = SingleJointPositionGoal()


def main():
    rospy.init_node('sanity_check_of_head_action_server')

    head = baxter_interface.Head()
    pos_init = head.pan()

    hc = HeadClient()
    if pos_init > (-1.39 + 0.3):
        hc.command(position=pos_init - 0.3, velocity=1.0)
        hc.wait()
    if pos_init < (1.39 - 0.3):
        hc.command(position=pos_init + 0.3, velocity=1.0)
        hc.wait()
    hc.command(position=pos_init, velocity=1.0)
    hc.wait()


if __name__ == '__main__':
    main()
