#!/usr/bin/env python

import rospy
import time, socket, os
from math import fabs
import threading
import re

from sound_play.libsoundplay import SoundClient

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from fetch_driver_msgs.msg import RobotState
from geometry_msgs.msg import Twist
from power_msgs.msg import BatteryState, BreakerState
from power_msgs.srv import BreakerCommand,  BreakerCommandRequest

## http://stackoverflow.com/questions/323972/is-there-any-way-to-kill-a-thread-in-python
import ctypes
def terminate_thread(thread):
    """Terminates a python thread from another thread.

    :param thread: a threading.Thread instance
    """
    if not thread.isAlive():
        return

    exc = ctypes.py_object(SystemExit)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(thread.ident), exc)
    if res == 0:
        raise ValueError("nonexistent thread id")
    elif res > 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")

class Warning:
    def __init__(self):
        time.sleep(1)
        self.robot_state_msgs = RobotState()
        self.battery_state_msgs = BatteryState()
        self.twist_msgs = Twist()
        #
        self.auto_undocking = False
        #
        self.base_breaker = rospy.ServiceProxy('base_breaker', BreakerCommand)
        #
        self.battery_sub = rospy.Subscriber("battery_state", BatteryState, self.battery_callback, queue_size = 1)
        self.cmd_vel_sub = rospy.Subscriber("base_controller/command_unchecked", Twist, self.cmd_vel_callback, queue_size = 1)
        self.robot_state_sub = rospy.Subscriber("robot_state", RobotState, self.robot_state_callback, queue_size = 1)
        self.undock_sub = rospy.Subscriber("/undock/status", GoalStatusArray, self.undock_status_callback)
        #
        self.cmd_vel_pub = rospy.Publisher("base_controller/command", Twist, queue_size=1)
        self.volume = rospy.get_param("~volume", 1.0)

    def undock_status_callback(self, msg):
        for status in msg.status_list:
            if status.status == GoalStatus.ACTIVE:
                self.auto_undocking = True
                return
        self.auto_undocking = False

    def robot_state_callback(self, msg):
        self.robot_state_msgs = msg

    def battery_callback(self, msg):
        self.battery_state_msgs = msg
        #
        if msg.is_charging == False and msg.charge_level < 0.1:
            sound.play(2, volume=self.volume)
            time.sleep(2)
            sound.play(5, volume=self.volume)
            time.sleep(5)

    def cmd_vel_callback(self, msg):
        ## warn when cmd_vel is issued while the robot is charning
        rospy.logdebug("cmd_vel : x:{} y:{} z:{}, battery.is_charning {}".format(msg.linear.x,msg.linear.y,msg.angular.z,self.battery_state_msgs.is_charging))
        breaker_enabled = True
        try:
            breaker_status = filter(lambda n: n.name=='base_breaker', self.robot_state_msgs.breakers)[0]
            breaker_enabled = breaker_status.state == BreakerState.STATE_ENABLED
        except Exception as e:
            rospy.logerr("Failed to fetch breaker status: %s" % str(e))

        if ( fabs(msg.linear.x) > 0 or fabs(msg.linear.y) > 0 or fabs(msg.angular.z) > 0 ) and \
           self.battery_state_msgs.is_charging == True and breaker_enabled and \
           self.auto_undocking != True:
            rospy.logerr("Try to run while charging!")
            self.base_breaker(BreakerCommandRequest(enable=False))
            sound.play(4, volume=self.volume) # play builtin sound Boom!
            time.sleep(5)
            self.base_breaker(BreakerCommandRequest(enable=True))
        else:
            self.cmd_vel_pub.publish(msg)
        ##
        self.twist_msgs = msg

if __name__ == "__main__":
    global sound
    # store error status and time of the error in the latest 10 minites
    global error_status_in_10_min
    error_status_in_10_min = {}
    rospy.init_node("cable_warning")
    sound = SoundClient()
    w = Warning()
    rospy.spin()
