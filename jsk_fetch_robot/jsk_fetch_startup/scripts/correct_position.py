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
        robot_name = rospy.get_param('/robot/name')
        if robot_name == 'fetch15':
            dock_name = '/eng2/7f/room73B2-fetch-dock-front'
        elif robot_name == 'fetch1075':
            dock_name = '/eng2/7f/room73B2-fetch-dock-entrance'
        for spot in spots.markers:
            if spot.text == dock_name:
                self.dock_pose = spot.pose

        self.is_docking = False
        self.timer = rospy.Timer(rospy.Duration(1.0), self._cb_correct_position)

    def subscribe(self):
        self.sub_dock = rospy.Subscriber(
            '/battery_state', BatteryState, self._cb)

    def unsubscribe(self):
        self.sub_dock.unregister()

    def _cb(self, msg):
        self.poke()
        self.is_docking = msg.is_charging

    def _cb_correct_position(self, event):
        if self.is_docking:
            self.pos = PoseWithCovarianceStamped()
            self.pos.header.stamp = rospy.Time.now()
            self.pos.header.frame_id = 'map'
            self.pos.pose.pose = self.dock_pose
            self.pos.pose.covariance = [1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 1e-3]
            self.pub.publish(self.pos)


if __name__ == '__main__':
    rospy.init_node('correct_position')
    spots = rospy.wait_for_message('/spots_marker_array', MarkerArray)
    cp = CorrectPosition()
    rospy.spin()
