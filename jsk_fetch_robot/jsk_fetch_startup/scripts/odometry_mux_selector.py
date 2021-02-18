#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rostopic

from topic_tools.srv import MuxSelectRequest

class OdometryMuxSelector(object):

    def __init__(self):

        try:
            self._rate_threshold = float(rospy.get_param('~rate_threshold', 10))
            self._topic_odom_initial = str(rospy.get_param('~topic_odom_initial', ''))
            self._topic_odom_backup = str(rospy.get_param('~topic_odom_backup', ''))
            self._topic_tf_initial = str(rospy.get_param('~topic_tf_initial', ''))
            self._topic_tf_backup = str(rospy.get_param('~topic_tf_backup', ''))
        except Exception as e:
            rospy.logerr('Error:{}'.format(e))

        rospy.wait_for_service('~select_service_topic')
        rospy.wait_for_service('~select_service_tf')
        self._srv_select_topic = rospy.ServiceProxy('~select_service_topic', MuxSelectRequest)
        self._srv_select_tf = rospy.ServiceProxy('~select_service_tf', MuxSelectRequest)

        self.r = rostopic.ROSTopicHz(-1)
        rospy.Subscriber(self._topic_odom_initial, rospy.AnyMsg, self.r.callback_hz, callback_args=self._topic_odom_initial)
        self._flag_backup = False

    def select(self, topic_odom, topic_tf):

        try:
            self._srv_topic(topic_odom)
            self._srv_tf(topic_tf)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('Service call failed: {}'.format(e))

    def spin(self):

        rate = rospy.sleep(1)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self._flag_backup and \
                    self.r.get_hz(self._topic_odom_initial)[0] < self._rate_threshold:
                rospy.logwarn('Publish rate of initial topic is low. switched to backup topics')
                self._flag_backup = True
                self.select( self._topic_odom_backup, self._topic_tf_backup )

def main():

    rospy.init_node('odometry_mux_selector')
    selector = OdometryMuxSelector()
    selector.spin()

if __name__ == '__main__':
    main()
