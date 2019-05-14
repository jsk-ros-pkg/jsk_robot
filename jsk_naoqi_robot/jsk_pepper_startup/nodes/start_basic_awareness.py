#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from subprocess import *
from std_srvs.srv import (
    SetBoolResponse,
    SetBool)

def start_basic_awareness_service(req):
    rospy.wait_for_service('/basic_awareness/set_enabled')
    basic_awareness_proxy = rospy.ServiceProxy('/basic_awareness/set_enabled', SetBool)
    basic_awareness_proxy(req)
    return SetBoolResponse()

def start_basic_awareness(req):
    # call set_enabled service
    start_basic_awareness_service(req)
    # kill basic awareness node
    call(['rosnode', 'kill', '/naoqi_basic_awareness'])

if __name__ == "__main__":
    rospy.init_node("start_basic_awareness")
    enabled = rospy.get_param("~enabled", True)
    start_basic_awareness(enabled)
