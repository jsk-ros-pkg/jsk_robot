#!/usr/bin/env python

import rospy
import time, socket, os
from math import fabs
import threading
import re

from sound_play.libsoundplay import SoundClient

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
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

class DiagnosticsSpeakThread(threading.Thread):
    def __init__(self, error_status):
        threading.Thread.__init__(self)
        self.error_status = error_status
        self.start()

    def run(self):
        global sound
        for status in  self.error_status:
            # we can ignore "Joystick not open."
            if status.message == "Joystick not open." :
                continue
            if status.name == "supply_breaker" and status.message == "Disabled." :
                continue
            #
            text = "Error on {}, {}".format(status.name, status.message)
            rospy.loginfo(text)
            text = text.replace('_', ', ')
            sound.say(text, 'voice_kal_diphone')
            time.sleep(5)

    def stop(self):
        terminate_thread(self)
        self.join()

class Warning:
    def __init__(self):
        time.sleep(1)
        self.battery_sub = rospy.Subscriber("battery_state", BatteryState, self.battery_callback, queue_size = 1)
        self.cmd_vel_sub = rospy.Subscriber("base_controller/command", Twist, self.cmd_vel_callback, queue_size = 1)
        self.robot_state_sub = rospy.Subscriber("robot_state", RobotState, self.robot_state_callback, queue_size = 1)
        self.diagnostics_status_sub = rospy.Subscriber("diagnostics", DiagnosticArray, self.diagnostics_status_callback, queue_size = 1)
        self.base_breaker = rospy.ServiceProxy('base_breaker', BreakerCommand)
        #
        self.battery_state_msgs = BatteryState()
        self.twist_msgs = Twist()
        #
        self.diagnostics_speak_thread = {}

    def robot_state_callback(self, msg):
        self.robot_state_msgs = msg

    def battery_callback(self, msg):
        self.battery_state_msgs = msg
        #
        if msg.is_charging == False and msg.charge_level < 0.1:
            sound.play(2)
            time.sleep(2)
            sound.play(5)
            time.sleep(5)

    def cmd_vel_callback(self, msg):
        ## warn when cmd_vel is issued while the robot is charning
        rospy.logdebug("cmd_vel : x:{} y:{} z:{}, battery.is_charning {}".format(msg.linear.x,msg.linear.y,msg.angular.z,self.battery_state_msgs.is_charging))
        breaker_status = filter(lambda n: n.name=='base_breaker', self.robot_state_msgs.breakers)[0]
        if ( fabs(msg.linear.x) > 0 or fabs(msg.linear.y) > 0 or fabs(msg.angular.z) > 0 ) and \
           self.battery_state_msgs.is_charging == True and breaker_status.state == BreakerState.STATE_ENABLED:
            rospy.logerr("Try to run while charging!")
            self.base_breaker(BreakerCommandRequest(enable=False))
            sound.play(4) # play builtin sound Boom!
            time.sleep(5)
            self.base_breaker(BreakerCommandRequest(enable=True))
        ##
        self.twist_msgs = msg

    def diagnostics_status_callback(self, msg):
        ##
        self.diagnostic_status_msgs = msg.status
        ##
        ## check if this comes from /robot_driver
        callerid = msg._connection_header['callerid']
        if not self.diagnostics_speak_thread.has_key(callerid):
            self.diagnostics_speak_thread[callerid] = None
        error_status = filter(lambda n: n.level in [DiagnosticStatus.WARN, DiagnosticStatus.ERROR, DiagnosticStatus.STALE], msg.status)
        # when RunStopped, ignore message from *_mcb and *_breaker
        if self.robot_state_msgs.runstopped:
            error_status = filter(lambda n: not (re.match("\w*_(mcb|breaker)",n.name) or (n.name == "Mainboard" and n.message == "Runstop pressed")), error_status)
        if not error_status : # error_status is not []
            if self.diagnostics_speak_thread[callerid] and self.diagnostics_speak_thread[callerid].is_alive():
                self.diagnostics_speak_thread[callerid].stop()
            return
        # make sure that diagnostics_speak_thread is None, when the thread is terminated
        if self.diagnostics_speak_thread[callerid] and not self.diagnostics_speak_thread[callerid].is_alive():
            self.diagnostics_speak_thread[callerid] = None
        # run new thread
        if self.diagnostics_speak_thread[callerid] is None:
            self.diagnostics_speak_thread[callerid] = DiagnosticsSpeakThread(error_status)

if __name__ == "__main__":
    global sound
    rospy.init_node("cable_warning")
    sound = SoundClient()
    w = Warning()
    rospy.spin()




