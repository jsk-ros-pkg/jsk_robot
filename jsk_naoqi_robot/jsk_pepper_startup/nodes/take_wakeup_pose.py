#! /usr/bin/env python
# -*- coding: utf-8 -*-

'''
disable AutonomousLife and servo on
'''

import rospy
from rospy import Duration
import actionlib
from std_msgs.msg import String
from std_srvs.srv import (
    EmptyResponse,
    Empty,
    Trigger)
from naoqi_bridge_msgs.msg import (
    JointTrajectoryAction,
    JointTrajectoryResult,
    JointTrajectoryFeedback,
    JointTrajectoryGoal
)
from naoqi_bridge_msgs.srv import (
    GetStringResponse,
    GetString,
    SetStringResponse,
    SetString,
)
from nao_interaction_msgs.srv import (
    SetAudioMasterVolume,
    SetAudioMasterVolumeRequest,
    SetAudioMasterVolumeResponse
)
import math
from trajectory_msgs.msg import JointTrajectoryPoint

class TakeWakeUpPose():
    def __init__(self):
        # pub
        self.pub = rospy.Publisher("/speech", String, queue_size=10)
        self.body_parts = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
                           'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
                           'HipRoll', 'HipPitch', 'KneePitch',
                           'HeadYaw', 'HeadPitch']

    def get_life_state(self):
        rospy.wait_for_service('/pepper_robot/pose/life/get_state')
        get_life_proxy = rospy.ServiceProxy('/pepper_robot/pose/life/get_state', Trigger)
        res_get_life = get_life_proxy()
        return res_get_life.message

    def disable_life(self):
        rospy.wait_for_service('/pepper_robot/pose/life/disable')
        disable_life_proxy = rospy.ServiceProxy('/pepper_robot/pose/life/disable', Empty)
        disable_life_proxy()
        return EmptyResponse()

    def servo_on(self):
        rospy.wait_for_service('/pepper_robot/pose/wakeup')
        servo_on_proxy = rospy.ServiceProxy('/pepper_robot/pose/wakeup', Empty)
        servo_on_proxy()
        return EmptyResponse()

    def set_language(self, language):
        rospy.wait_for_service('/naoqi_driver/get_language')
        get_language_proxy = rospy.ServiceProxy('/naoqi_driver/get_language', GetString)
        current_language = get_language_proxy()
        if current_language != language:
            rospy.wait_for_service('/naoqi_driver/set_language')
            set_language_proxy = rospy.ServiceProxy('/naoqi_driver/set_language', SetString)
            set_language_proxy(language)

    def set_volume(self, volume):
        rospy.wait_for_service('/naoqi_driver/set_volume')
        set_volume_proxy = rospy.ServiceProxy('/naoqi_driver/set_volume', SetAudioMasterVolume)
        vol = SetAudioMasterVolumeRequest()
        vol.master_volume.data = volume
        set_volume_proxy(vol)
        return SetAudioMasterVolumeResponse()

    def movement (self, goal):
        # reference: https://github.com/ros-naoqi/nao_robot/blob/master/nao_apps/scripts/test_joint_angles.py
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient('/pepper_robot/pose/joint_trajectory', JointTrajectoryAction)
        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()
        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        return client.get_result()

    def deg_to_rad (self, deg):
        return math.radians(deg)

    def reset_pose (self):
        # Creates a goal to send to the action server.
        goal = JointTrajectoryGoal()
        # http://doc.aldebaran.com/2-5/family/pepper_technical/bodyparts_pep.html
        goal.trajectory.joint_names = self.body_parts
        angle_list_deg = [85.0, 10.0, -70.0, -20.0, -40.0,
                          85.0, -10.0, 70.0, 20.0, 40.0,
                          -2.0, -5.0, 2.0,
                          0.0, 0.0]
        angle_list_rad =map(self.deg_to_rad, angle_list_deg)
        goal.trajectory.points = []
        goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(4.0),
                                                           positions = angle_list_rad))
        self.movement(goal)

    def stretch_pose (self):
        # Creates a goal to send to the action server.
        goal = JointTrajectoryGoal()
        # http://doc.aldebaran.com/2-5/family/pepper_technical/bodyparts_pep.html
        goal.trajectory.joint_names = self.body_parts
        angle_list_deg = [-40.0, 40.0, -70.0, -20.0, -40.0,
                          -40.0, -40.0, 70.0, 20.0, 40.0,
                          -2.0, 5.0, 2.0,
                          0.0, -10.0]
        angle_list_rad =map(self.deg_to_rad, angle_list_deg)
        goal.trajectory.points = []
        goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(4.0),
                                                           positions = angle_list_rad))
        angle_list_deg = [-60.0, 60.0, -70.0, -20.0, -40.0,
                          -60.0, -60.0, 70.0, 20.0, 40.0,
                          -2.0, 15.0, 2.0,
                          5.0, -20.0]
        angle_list_rad =map(self.deg_to_rad, angle_list_deg)
        goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(7.0),
                                                           positions = angle_list_rad))

        self.movement(goal)

    def take_initial_pose(self):
        try:
            # check current AutonomousLife status
            life_state = self.get_life_state()
            # set AutonomousLife disabled
            if not(life_state == "disabled"):
                self.disable_life()
            # servo on
            self.servo_on()
            rospy.sleep(1)
            # speak
            self.set_volume(40)
            self.set_language("Japanese")
            msg = String()
            msg.data = "\\rspd=40\\\\vct=140\\ムニャムニャ\\vct=120\\\\rspd=100\\"
            self.pub.publish(msg)
            # stretch
            self.stretch_pose()
            # speak
            msg = String()
            msg.data = "\\rspd=40\\\\vct=140\\ファーーッ\\vct=120\\\\rspd=100\\"
            self.pub.publish(msg)
            rospy.sleep(1)
            self.reset_pose()
            # speak
            msg = String()
            msg.data = "\\vct=140\\よしっ\\vct=120\\\\rspd=100\\"
            self.pub.publish(msg)

        except RuntimeError as e:
            rospy.logerr("Exception caught:\n%s", e)
                
if __name__=="__main__":
    rospy.init_node("take_initial_pose")
    take_wakeup_pose = TakeWakeUpPose()
    take_wakeup_pose.take_initial_pose()
    exit(0)
