#! /usr/bin/env python
# -*- coding: utf-8 -*-

'''
disable AutonomousLife and servo on
'''

import rospy
from std_srvs.srv import (
    EmptyResponse,
    Empty,
    Trigger)

def get_life_state():
    rospy.wait_for_service('/pepper_robot/pose/life/get_state')
    get_life_proxy = rospy.ServiceProxy('/pepper_robot/pose/life/get_state', Trigger)
    res_get_life = get_life_proxy()
    return res_get_life.message

def disable_life():
    rospy.wait_for_service('/pepper_robot/pose/life/disable')
    disable_life_proxy = rospy.ServiceProxy('/pepper_robot/pose/life/disable', Empty)
    disable_life_proxy()
    return EmptyResponse()

def servo_on():
    rospy.wait_for_service('/pepper_robot/pose/wakeup')
    servo_on_proxy = rospy.ServiceProxy('/pepper_robot/pose/wakeup', Empty)
    servo_on_proxy()
    return EmptyResponse()

def take_initial_pose():
    try:
        # check current AutonomousLife status
        life_state = get_life_state()
        # set AutonomousLife disabled
        if not(life_state == "disabled"):
            disable_life()
        # servo on
        servo_on()
    except RuntimeError, e:
        rospy.logerr("Exception caught:\n%s", e)
                
if __name__=="__main__":
    rospy.init_node("take_initial_pose")
    take_initial_pose()
    exit(0)
