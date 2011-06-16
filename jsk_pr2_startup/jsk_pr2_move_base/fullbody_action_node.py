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
        self._torso_client=actionlib.SimpleActionClient('/torso_controller/position_joint_action', SingleJointPositionAction)
        self._head_client=actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
#        self._base_client=actionlib.SimpleActionClient('/base_trajectory_action', JointTrajectoryAction)
        self._clients = [self._larm_client, self._rarm_client,
                         self._torso_client, self._head_client] #, self._base_client]
        for client in self._clients:
            client.wait_for_server()

        self._head_thread = Thread(target = self.head)
        self._head_thread.start()
        self._torso_thread = Thread(target = self.torso)
        self._torso_thread.start()

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

    def torso(self):
        while not rospy.is_shutdown():
            self._lockobj.acquire()
            sleep_sec = 0.02

            try:
                valids = [x for x in self._trajectories if -0.05 < (rospy.Time.now()-x.header.stamp).to_sec() < x.points[0].time_from_start.to_sec() - 0.05]
                traj = valids[0]
                indexes = [x[0] for x in enumerate(traj.joint_names)
                           if x[1] == "torso_lift_joint"]
                pts = [(pt.positions[indexes[0]], pt.time_from_start + traj.header.stamp) for pt in traj.points]
                pts = [x for x in pts if rospy.Time.now() + rospy.Duration(0.02) < x[1]]

                if 0 < len(pts):
                    pt = pts[0]
                    jgoal = SingleJointPositionGoal()
                    jgoal.position = pt[0]
                    jgoal.min_duration = pt[1] - rospy.Time.now()
                    jgoal.max_velocity = 1.0
                    self._torso_client.send_goal(jgoal)
                    sleep_sec = jgoal.min_duration.to_sec()-0.01
            except:
                #print "Unexpected error:", sys.exc_info()
                None
            self._lockobj.release()
            rospy.sleep(sleep_sec)

    def head(self):
        while not rospy.is_shutdown():
            self._lockobj.acquire()
            sleep_sec = 0.02

            try:
                valids = [x for x in self._trajectories if -0.05 < (rospy.Time.now()-x.header.stamp).to_sec() < x.points[0].time_from_start.to_sec() - 0.05]
                traj = valids[0]
                joints = ["head_pan_joint", "head_tilt_joint"]
                indexes = [x[0] for y in joints for x in enumerate(traj.joint_names) if x[1] == y]
                pts = [(pt.positions[indexes[0]], pt.positions[indexes[1]], pt.time_from_start + traj.header.stamp) for pt in traj.points]
                pts = [x for x in pts if rospy.Time.now() + rospy.Duration(0.02) < x[2]]
                if 0 < len(pts):
                    pt = pts[0]
                    target_x = math.cos(pt[0])
                    target_y = math.sin(pt[0])
                    target_z = - math.tan(pt[1])
                    jgoal = PointHeadGoal()
                    jgoal.target = PointStamped()
                    jgoal.target.header.stamp = traj.header.stamp
                    jgoal.target.header.frame_id = "/torso_lift_link"
                    jgoal.target.point = Point(x=target_x, y=target_y,
                                               z=target_z + 0.3875)
                    jgoal.min_duration = pt[2] - rospy.Time.now()
                    jgoal.max_velocity = 1.0 # ??
                    self._head_client.send_goal(jgoal)
                    sleep_sec = jgoal.min_duration.to_sec()-0.01
            except:
                None
            self._lockobj.release()
            rospy.sleep(sleep_sec)

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

    def add_trajectory(self, traj):
        # TODO sort and check
        self._lockobj.acquire()
        self._trajectories = [pt for pt in self._trajectories
                              if 0 < (traj.header.stamp-pt.header.stamp).to_sec()] + [JointTrajectory(header=traj.header, joint_names=traj.joint_names, points=[pt]) for pt in traj.points]
        self._lockobj.release()

    def execute_cb(self, goal):
        self.add_trajectory(goal.trajectory)

        self.larm(goal.trajectory)
        self.rarm(goal.trajectory)
#       self.base(goal.trajectory)

        self._as.set_succeeded(JointTrajectoryResult())

if __name__ == "__main__":
    rospy.init_node('fullbody_controller')
    action = FullbodyAction(rospy.get_name())
    rospy.spin()
