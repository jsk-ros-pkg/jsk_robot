#!/usr/bin/env python
###############################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, Kei OKada
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
import tf
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Pose
from unitree_legged_msgs.msg import HighCmd

'''
uint8 mode                      # 0. idle, default stand
                                # 1. force stand (controlled by dBodyHeight + ypr)
                                # 2. target velocity walking (controlled by velocity + yawSpeed)
                                # 3. target position walking (controlled by position + ypr[0])
                                # 4. path mode walking (reserve for future release)
                                # 5. position stand down.
                                # 6. position stand up
                                # 7. damping mode
                                # 8. recovery stand
                                # 9. backflip
                                # 10. jumpYaw
                                # 11. straightHand
                                # 12. dance1
                                # 13. dance2
                                # 14. two leg stand
uint8 gaitType                  # 0.idle  1.trot  2.trot running  3.climb stair
uint8 speedLevel                # 0. default low speed. 1. medium speed 2. high speed. during walking, only
float32 footRaiseHeight         # (unit: m, default: 0.08m), foot up height while walking
float32 bodyHeight              # (unit: m, default: 0.28m),
float32[2] postion              # (unit: m), desired position in inertial frame
float32[3] euler                # (unit: rad), roll pitch yaw in stand mode
float32[2] velocity             # (unit: m/s), forwardSpeed, sideSpeed in body frame
float32 yawSpeed                # (unit: rad/s), rotateSpeed in body frame
BmsCmd bms
LED[4] led
uint8[40] wirelessRemote
uint32 reserve                  # Old version Aliengo does not have
'''

PKG = 'jsk_unitree_startup'
NAME = 'unitree_service'

class UnitreeService(object):
    def __init__(self):
        self.stand_service = rospy.Service("stand", Trigger, self.stand)
        self.sit_service = rospy.Service("sit", Trigger, self.sit)
        self.bodypose_sub = rospy.Subscriber("body_pose", Pose, self.body_pose);

        self.highlevel_pub = rospy.Publisher("/high_cmd", HighCmd);

    def stand(self, req):
        msg = HighCmd()
        msg.mode = 6
        self.highlevel_pub.publish(msg)
        return TriggerResponse(success=True)

    def sit(self, req):
        msg = HighCmd()
        msg.mode = 5
        self.highlevel_pub.publish(msg)
        return TriggerResponse(success=True)

    def body_pose(self, pose):
        q = pose.orientation
        msg = HighCmd()
        msg.mode = 1
        msg.euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]) # RPY
        self.highlevel_pub.publish(msg)
        return


if __name__ == "__main__":
    rospy.init_node("unitree_service")
    n = UnitreeService()
    rospy.spin()
