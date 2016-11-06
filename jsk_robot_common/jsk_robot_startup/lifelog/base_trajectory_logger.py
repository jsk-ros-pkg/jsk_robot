#!/usr/bin/python

import rospy
from jsk_robot_startup.lifelog import BaseTrajectoryLogger


if __name__ == "__main__":
    rospy.init_node('base_trajectory_logger')
    BaseTrajectoryLogger().run()
