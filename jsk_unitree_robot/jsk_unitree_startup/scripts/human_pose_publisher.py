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

import rospy
from jsk_recognition_msgs.msg import PeoplePose
from jsk_recognition_msgs.msg import PeoplePoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from paho.mqtt import client as mqtt_client

class HumanPosePublisher(object):
    broker = '192.168.123.161'
    port = 1883
    topic = "vision/human_pose"
    last_received = None


    def __init__(self):
        rospy.init_node("human_pose_publisher")
        self.pub = rospy.Publisher("/people_pose", PeoplePoseArray, queue_size=10);
        self.connect_mqtt()
        self.subscribe()
        self.client.loop_start()
        self.last_received = rospy.Time.now()
        rospy.Timer(rospy.Duration(5), self.timer_cb)

    def timer_cb(self, msg):
        if self.last_received and (rospy.Time.now() - self.last_received).to_sec() > 5.0:
            rospy.loginfo("We haven't received any MQTT message for 5 sec, start AI camera")
            self.client.publish('vision/ai_mode', 'cam1')

    def connect_mqtt(self):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker! ({}:{})".format(self.broker, self.port))
            else:
                print("Failed to connect, return code %d\n", rc)

        self.client = mqtt_client.Client(rospy.get_name())
        self.client.on_connect = on_connect
        self.client.connect(self.broker, self.port)
        return


    def subscribe(self):
        def on_message(client, userdata, msg):
            # keep message received time to start AI mode
            self.last_received = rospy.Time.now()

            rospy.loginfo("Received `{}` from `{}` topic".format(msg.payload.decode(), msg.topic))
            poses_msg = PeoplePoseArray()
            for topic in eval(msg.payload.decode()):  # convert str to list by eval()
                pose_msg = PeoplePose()
                for limb, pos in topic.items():
                    pose_msg.limb_names.append(limb)
                    pose_msg.poses.append(
                        Pose(position=Point(x=pos[0], y=pos[1])))
                poses_msg.poses.append(pose_msg)
            self.pub.publish(poses_msg)

        self.client.subscribe(self.topic)
        self.client.on_message = on_message
        return


if __name__ == "__main__":
    rospy.init_node("human_pose_publisher")
    n = HumanPosePublisher()
    rospy.spin()
