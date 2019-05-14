#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from subprocess import *
from std_srvs.srv import (
    SetBoolResponse,
    SetBool)

def start_background_movement_service(req):
    rospy.wait_for_service('/background_movement/set_enabled')
    background_movement_proxy = rospy.ServiceProxy('/background_movement/set_enabled', SetBool)
    background_movement_proxy(req)
    return SetBoolResponse()

def start_background_movement(req):
    # call set_enabled service
    start_background_movement_service(req)
    # kill background movement node
    call(['rosnode', 'kill', '/naoqi_background_movement'])

if __name__ == "__main__":
    rospy.init_node("start_background_movement")
    enabled = rospy.get_param("~enabled", True)
    start_background_movement(enabled)
