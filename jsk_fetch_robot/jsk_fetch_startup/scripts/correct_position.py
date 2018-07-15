#!/usr/bin/env python

import rospy

from jsk_topic_tools import ConnectionBasedTransport
from power_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray


class CorrectPosition(ConnectionBasedTransport):

    def __init__(self):
        super(CorrectPosition, self).__init__()
        self.pub = self.advertise('initialpose',
                                  PoseWithCovarianceStamped,
                                  queue_size=1)
        self.dock_pose = Pose()
        for spot in spots.markers:
            if spot.text == '/eng2/7f/room73B2-fetch-dock-front':
                self.dock_pose = spot.pose

        self.is_docking = False

    def subscribe(self):
        self.sub_dock = rospy.Subscriber('/battery_state',
                                         BatteryState,
                                         self._cb_correct_position)

    def unsubscribe(self):
        self.sub_dock.unregister()

    def _cb_correct_position(self, msg):
        if msg.is_charging and (not self.is_docking):
            self.pos = PoseWithCovarianceStamped()
            self.pos.header.stamp = rospy.Time.now()
            self.pos.header.frame_id = 'map'
            self.pos.pose.pose = self.dock_pose
            self.pos.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
            self.pub.publish(self.pos)
        self.is_docking = msg.is_charging


if __name__ == '__main__':
    rospy.init_node('correct_position')
    spots = rospy.wait_for_message('/spots_marker_array', MarkerArray)
    cp = CorrectPosition()
    rospy.spin()
