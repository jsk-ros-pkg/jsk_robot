#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from pr2_msgs.msg import PowerBoardState
from std_msgs.msg import Bool
from std_srvs.srv import Empty


class PR2ResetMotorsNode(object):
    def __init__(self):
        # By default, motor will be reset 3 times maximum in 5 minutes
        self.max_retry_num = rospy.get_param("~max_retry_num", 3)
        self.watch_duration = rospy.get_param("~watch_duration", 300)

        self.history = []
        self.run_stop = False
        self.calibrated = False

        self.reset_srv = rospy.ServiceProxy("/pr2_ethercat/reset_motors", Empty)
        self.sub_runstop = rospy.Subscriber("/power_board/state", PowerBoardState, self.runstop_cb)
        self.sub_motors  = rospy.Subscriber("/pr2_ethercat/motors_halted", Bool, self.motors_cb)
        self.sub_calib   = rospy.Subscriber("/calibrated", Bool, self.calib_cb)

    def calib_cb(self, msg):
        self.calibrated = msg.data

    def runstop_cb(self, msg):
        self.run_stop = msg.run_stop

    def motors_cb(self, msg):
        if not self.calibrated:
            return
        halted = msg.data
        rospy.logdebug("runstop: %s, halted: %s" % (self.run_stop, halted))
        if self.run_stop and halted:
            rospy.logwarn("motor halted, but run stop is true")
            stamp = rospy.Time.now()
            history = filter(
                lambda s: (stamp-s).to_sec() < self.watch_duration,
                self.history)
            if len(history) > self.max_retry_num:
                rospy.logerr("Maximum retry count reached. Give up resetting motors")
                return
            rospy.logwarn("resetting motors")
            self.reset_srv()
            self.history = history + [stamp]


if __name__ == '__main__':
    rospy.init_node("pr2_reset_motors")
    n = PR2ResetMotorsNode()
    rospy.spin()
