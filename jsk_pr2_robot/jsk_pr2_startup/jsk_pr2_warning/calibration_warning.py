#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import actionlib
import diagnostic_msgs.msg
import sound_play.msg


RUNSTOP_MESSAGE = 'Calibration is on hold because '\
    'motors are halted. Enable the run-stop'
RUNSTOP_SPEAK_MESSAGE = \
    u"キャリブレーションをするためにランストップをオンにしてください"
STUCK_JOINT_SPEAK_MESSAGE = u"%sがスタックしました。アシストしてください。"


class CalibrationWarning(object):

    def __init__(self):
        self.actionclient = actionlib.SimpleActionClient(
            '/robotsound_jp',
            sound_play.msg.SoundRequestAction)
        self.actionclient.wait_for_server()

        self.diagnostics_status_sub = rospy.Subscriber(
            "diagnostics",
            diagnostic_msgs.msg.DiagnosticArray,
            self.diagnostics_status_callback,
            queue_size=1)

    def diagnostics_status_callback(self, msg):
        for status in msg.status:
            if status.name == 'Calibration on hold':
                self.speak_warning_message(RUNSTOP_SPEAK_MESSAGE)
            elif status.name == 'Calibration stuck':
                # Assuming message is following.
                # 'Joint %s is taking a long time to calibrate.
                #  It might be stuck and need some human help'
                s = status.message
                stuck_joint_names = s[6:s.index(' is taking')]
                self.speak_warning_message(
                    STUCK_JOINT_SPEAK_MESSAGE % stuck_joint_names)

    def speak_warning_message(self, warn_message):
        msg = sound_play.msg.SoundRequest()
        msg.sound = sound_play.msg.SoundRequest.SAY
        msg.command = sound_play.msg.SoundRequest.PLAY_ONCE
        msg.arg = warn_message
        msg.volume = 1.0

        goal = sound_play.msg.SoundRequestGoal()
        goal.sound_request = msg
        self.actionclient.send_goal(goal)
        self.actionclient.wait_for_result()


if __name__ == '__main__':
    rospy.init_node('pr2_calibration_warning')
    CalibrationWarning()
    rospy.spin()
