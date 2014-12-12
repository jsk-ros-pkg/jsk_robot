#!/usr/bin/env python
# -*- mode: python -*-

import roslib; roslib.load_manifest('setup_fullbody_controller')
import rospy
import actionlib

import math
import thread
from threading import Thread

from geometry_msgs.msg import PointStamped, Point
from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import *

class FullbodyAction(object):
    def __init__(self, name):
        self._action_name = name + '/joint_trajectory_action'
        self._trajectories = [] # list of JointTrajectory
        self._lockobj = thread.allocate_lock()
        self._as = actionlib.SimpleActionServer(self._action_name, JointTrajectoryAction, execute_cb=self.execute_cb)

        # controllers
        self._larm_client=actionlib.SimpleActionClient('/l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        self._rarm_client=actionlib.SimpleActionClient('/r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        self._torso_client=actionlib.SimpleActionClient('/torso_controller/joint_trajectory_action', JointTrajectoryAction)
        self._head_client=actionlib.SimpleActionClient('/head_traj_controller/head_trajectory_action', JointTrajectoryAction)
        self._base_client=actionlib.SimpleActionClient('/base_controller/joint_trajectory_action', JointTrajectoryAction)
        self._clients = [self._larm_client, self._rarm_client,
                         self._torso_client, self._head_client,
                         self._base_client]
        for client in self._clients:
            client.wait_for_server()

        print 'fullbody action initialized'

    # action methods
    def larm(self, traj):
        joints = ["l_shoulder_pan_joint",
                  "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
                  "l_elbow_flex_joint", "l_forearm_roll_joint",
                  "l_wrist_flex_joint", "l_wrist_roll_joint"]
        return self.joint(self._larm_client, traj, joints)
    def rarm(self, traj):
        joints = ["r_shoulder_pan_joint",
                  "r_shoulder_lift_joint", "r_upper_arm_roll_joint",
                  "r_elbow_flex_joint", "r_forearm_roll_joint",
                  "r_wrist_flex_joint", "r_wrist_roll_joint"]
        return self.joint(self._rarm_client, traj, joints)
    def base(self, traj):
        joints = ["base_link_x", "base_link_y", "base_link_pan"]
        return self.joint(self._base_client, traj, joints)
    def torso(self, traj):
        joints = ["torso_lift_joint"]
        return self.joint(self._torsi_client, traj, joints)
    def head(self, traj):
        joints = ["head_pan_joint", "head_tilt_joint"]
        return self.joint(self._head_client, traj, joints)

    def joint(self, client, traj, joints):
        try:
            indexes = [y[0] for x in joints for y in enumerate(traj.joint_names) if x == y[1]]
            if len(indexes) != len(joints): return

            jgoal = JointTrajectory()
            jgoal.header.stamp = traj.header.stamp
            jgoal.joint_names = joints
            jgoal.points=[JointTrajectoryPoint(
                    time_from_start=pt.time_from_start,
                    positions=[pt.positions[i] for i in indexes if pt.positions != []],
                    velocities=[pt.velocities[i] for i in indexes if pt.velocities != []])
                          for pt in traj.points]
            client.send_goal(JointTrajectoryGoal(trajectory=jgoal))
        except:
            print 'error in joint client'

    def execute_cb(self, goal):
        self.larm(goal.trajectory)
        self.rarm(goal.trajectory)
        self.base(goal.trajectory)
        self.torso(goal.trajectory)
        self.head(goal.trajectory)

        self._as.set_succeeded(JointTrajectoryResult())

if __name__ == "__main__":
    rospy.init_node('fullbody_controller')
    action = FullbodyAction(rospy.get_name())
    rospy.spin()
