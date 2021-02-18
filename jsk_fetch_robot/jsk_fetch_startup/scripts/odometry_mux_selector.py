#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

import rospy
import rostopic

from topic_tools.srv import MuxSelect, MuxSelectRequest
from nav_msgs.msg import Odometry

class OdometryMuxSelector(object):

    def __init__(self):

        try:
            self._topic_odom_initial = str(rospy.get_param('~topic_odom_initial', ''))
            self._topic_odom_backup = str(rospy.get_param('~topic_odom_backup', ''))
            self._topic_tf_initial = str(rospy.get_param('~topic_tf_initial', ''))
            self._topic_tf_backup = str(rospy.get_param('~topic_tf_backup', ''))
            self._duration_timeout_topic = float(rospy.get_param('~duration_timeout_topic',10.0))
        except Exception as e:
            rospy.logerr('Error:{}'.format(e))
            sys.exit(1)

        try:
            rospy.wait_for_service('~select_service_topic', 30.0)
            rospy.wait_for_service('~select_service_tf', 30.0)
        except rospy.ROSException as e:
            rospy.logerr('Service is not found:{}'.format(e))
            sys.exit(1)

        try:
            rospy.wait_for_message(self._topic_odom_initial,Odometry,self._duration_timeout_topic)
            rospy.wait_for_message(self._topic_odom_backup,Odometry,self._duration_timeout_topic)
        except rospy.ROSException as e:
            rospy.logwarn('Message is not published:{}'.format(e))

        self._srv_select_topic = rospy.ServiceProxy('~select_service_topic', MuxSelect)
        self._srv_select_tf = rospy.ServiceProxy('~select_service_tf', MuxSelect)

        self.r = rostopic.ROSTopicHz(-1)
        rospy.Subscriber(self._topic_odom_initial, rospy.AnyMsg, self.r.callback_hz, callback_args=self._topic_odom_initial)
        self._flag_backup = False

        rospy.loginfo('odometry_mux_selector is up')

    def select(self, topic_odom, topic_tf):

        try:
            self._srv_select_topic(topic_odom)
            self._srv_select_tf(topic_tf)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('Service call failed: {}'.format(e))

    def spin(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            try:
                rospy.wait_for_message(self._topic_odom_initial,Odometry,self._duration_timeout_topic)
            except rospy.ROSException as e:
                if not self._flag_backup:
                    rospy.logwarn('Initial topic "{}" seems not to be published. switched to backup topics'.format(self._topic_odom_initial))
                    self._flag_backup = True
                    self.select( self._topic_odom_backup, self._topic_tf_backup )

def main():

    rospy.init_node('odometry_mux_selector')
    selector = OdometryMuxSelector()
    selector.spin()

if __name__ == '__main__':
    main()
