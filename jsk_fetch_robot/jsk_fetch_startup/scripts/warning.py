#!/usr/bin/env python

import rospy
import time, socket, os
from math import fabs
import threading
import re

from sound_play.libsoundplay import SoundClient

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
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
            # ignore error status if the error already occured in the latest 10 minites
            if error_status_in_10_min.has_key(status.message):
                if rospy.Time.now().secs - error_status_in_10_min[status.message] < 600:
                    continue
                else:
                    error_status_in_10_min[status.message] = rospy.Time.now().secs
            else:
                error_status_in_10_min[status.message] = rospy.Time.now().secs
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
        self.robot_state_msgs = RobotState()
        self.battery_state_msgs = BatteryState()
        self.twist_msgs = Twist()
        #
        self.diagnostics_speak_thread = {}
        self.auto_undocking = False
        self.diagnostics_list = []
        if rospy.get_param("~speak_warn", True):
            self.diagnostics_list.append(DiagnosticStatus.WARN)
        if rospy.get_param("~speak_error", True):
            self.diagnostics_list.append(DiagnosticStatus.ERROR)
        if rospy.get_param("~speak_stale", True):
            self.diagnostics_list.append(DiagnosticStatus.STALE)
        #
        self.base_breaker = rospy.ServiceProxy('base_breaker', BreakerCommand)
        #
        self.battery_sub = rospy.Subscriber("battery_state", BatteryState, self.battery_callback, queue_size = 1)
        self.cmd_vel_sub = rospy.Subscriber("base_controller/command_unchecked", Twist, self.cmd_vel_callback, queue_size = 1)
        self.robot_state_sub = rospy.Subscriber("robot_state", RobotState, self.robot_state_callback, queue_size = 1)
        self.diagnostics_status_sub = rospy.Subscriber("diagnostics", DiagnosticArray, self.diagnostics_status_callback, queue_size = 1)
        self.undock_sub = rospy.Subscriber("/undock/status", GoalStatusArray, self.undock_status_callback)
        #
        self.cmd_vel_pub = rospy.Publisher("base_controller/command", Twist, queue_size=1)

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
            sound.play(2)
            time.sleep(2)
            sound.play(5)
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
            sound.play(4) # play builtin sound Boom!
            time.sleep(5)
            self.base_breaker(BreakerCommandRequest(enable=True))
        else:
            self.cmd_vel_pub.publish(msg)
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
        error_status = filter(lambda n: n.level in self.diagnostics_list, msg.status)
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
    # store error status and time of the error in the latest 10 minites
    global error_status_in_10_min
    error_status_in_10_min = {}
    rospy.init_node("cable_warning")
    sound = SoundClient()
    w = Warning()
    rospy.spin()
