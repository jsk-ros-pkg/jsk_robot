# -*- coding: utf-8 -*-

from jsk_spot_behavior_manager.base_behavior import BaseBehavior

import actionlib
import roslaunch
import rospkg
import rospy

import math

from switchbot_ros.msg import SwitchBotCommandGoal, SwitchBotCommandAction
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32, Bool, Int16


class ElevatorBehavior(BaseBehavior):

    def door_point_cloud_callback(self, msg):
        if len(msg.data) == 0:
            self.door_is_open = True
        else:
            self.door_is_open = False

    def current_floor_callback(self, msg):
        self.current_floor = msg.data

    def rest_elevator_callback(self, msg):
        self.rest_elevator = msg.data

    def run_initial(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_initial() called')

        self.silent_mode = rospy.get_param('~silent_mode', True)

        start_floor = start_node.properties['floor']

        # launch recognition launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        roslaunch_path = rospkg.RosPack().get_path('spot_basic_behaviors') +\
            '/launch/elevator_detection.launch'
        roslaunch_cli_args = [roslaunch_path, "initial_floor:={}".format(start_floor)]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
            roslaunch_cli_args)
        self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
            uuid,
            roslaunch_file
        )
        self.roslaunch_parent.start()

        # value for door openring checker
        self.door_is_open = False
        self.subscriber_door_check = None

        # elevator state
        self.current_floor = None
        self.rest_elevator = None

        self.subscriber_current_floor = rospy.Subscriber("/elevator_state_publisher/current_floor", Int16, self.current_floor_callback)
        self.subscriber_rest_elevator = rospy.Subscriber("/elevator_state_publisher/rest_elevator", Bool, self.rest_elevator_callback)

        # value for switchbot
        self.action_client_switchbot = actionlib.SimpleActionClient(
            '/switchbot_ros/switch',
            SwitchBotCommandAction
        )

        if not self.action_client_switchbot.wait_for_server(rospy.Duration(30)):
            rospy.logerr('switchbot server seems to fail.')
            return False

        try:
            rospy.wait_for_message('/spot_recognition/elevator_door_points', timeout=rospy.Duration(30))
            rospy.wait_for_message("/elevator_state_publisher/current_floor", timeout=rospy.Duration(30))
            rospy.wait_for_message("/elevator_state_publisher/rest_elevator", timeout=rospy.Duration(30))
        except rospy.ROSException:
            rospy.logerr("Some topics are not published.")
            return False

        return True

    def run_main(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_main() called')

        graph_name = edge.properties['graph']
        start_id = list(filter(
            lambda x: x['graph'] == graph_name,
            start_node.properties['waypoints_on_graph']
        ))[0]['id']
        end_id = edge.properties['goal_waypoint_id']
        rest_waypoint_id = edge.properties['rest_waypoint_id']
        localization_method = list(filter(
            lambda x: x['graph'] == graph_name,
            start_node.properties['waypoints_on_graph']
        ))[0]['localization_method']

        end_floor = end_node.properties['floor']

        # graph uploading and localization
        if pre_edge is not None and \
                graph_name == pre_edge.properties['graph']:
            rospy.loginfo('graph upload and localization skipped.')
        else:
            # Upload
            ret = self.spot_client.upload_graph(graph_name)
            if ret[0]:
                rospy.loginfo('graph {} uploaded.'.format(graph_name))
            else:
                rospy.logerr('graph uploading failed: {}'.format(ret[1]))
                return False
            # Localization
            if localization_method == 'fiducial':
                ret = self.spot_client.set_localization_fiducial()
            elif localization_method == 'waypoint':
                ret = self.spot_client.set_localization_waypoint(start_id)
            else:
                ret = (False, 'Unknown localization method')
            if ret[0]:
                rospy.loginfo('robot is localized on the graph.')
            else:
                rospy.logwarn('Localization failed: {}'.format(ret[1]))
                return False

        # start door opening check from outside
        self.subscriber_door_check = rospy.Subscriber(
            '/spot_recognition/elevator_door_points',
            PointCloud2,
            self.door_point_cloud_callback)

        # push button with switchbot
        rospy.loginfo('calling elevator when riding...')
        success_calling = False
        switchbot_goal = SwitchBotCommandGoal()
        switchbot_goal.device_name = start_node.properties['switchbot_device']
        switchbot_goal.command = 'press'
        count = 0
        while True:
            self.action_client_switchbot.send_goal(switchbot_goal)
            if self.action_client_switchbot.wait_for_result(rospy.Duration(10)):
                break
            count += 1
        if count >= 3:
            rospy.logerr('switchbot calling failed.')
            return False
        result = self.action_client_switchbot.get_result()
        rospy.loginfo('switchbot result: {}'.format(result))
        if not result.done:
            rospy.logerr('switchbot calling failed.')
            return False
        rospy.loginfo('elevator calling when riding on has succeeded')

        # wait for elevator
        rate = rospy.Rate(1)
        door_open_count = 0
        while not rospy.is_shutdown():
            rate.sleep()
            if self.door_is_open:
                door_open_count += 1
            else:
                door_open_count = 0
            if door_open_count >= 2:
                break
        rospy.loginfo('elevator door opened.')
        self.subscriber_door_check.unregister()
        self.subscriber_door_check = None

        # start navigation to rest point
        rate = rospy.Rate(10)
        if not self.silent_mode:
            self.sound_client.say('エレベータに乗り込みます。ご注意ください。', blocking=False)
        self.spot_client.navigate_to(rest_waypoint_id, blocking=False)
        # call elevator from destination floor while
        rospy.loginfo('calling elevator when getting off...')
        switchbot_goal = SwitchBotCommandGoal()
        switchbot_goal.device_name = end_node.properties['switchbot_device']
        switchbot_goal.command = 'press'
        self.action_client_switchbot.send_goal(switchbot_goal)
        ##
        if not self.action_client_switchbot.wait_for_result(timeout=rospy.Duration(20)):
            rospy.logerr('Switchbot timeout')
            self.spot_client.wait_for_navigate_to_result()
            self.spot_client.navigate_to(start_id, blocking=True)
            self.spot_client.wait_for_navigate_to_result()
            return False
        result_switchbot = self.action_client_switchbot.get_result()
        self.spot_client.wait_for_navigate_to_result()
        result_navigation = self.spot_client.get_navigate_to_result()
        # recovery when riding on
        if not result_navigation.success or not result_switchbot.done:
            rospy.logerr('Failed to ride on a elevator. result_navigation: {}, result_switchbot: {}'.format(
                result_navigation, result_switchbot))
            self.spot_client.navigate_to(start_id, blocking=True)
            self.spot_client.wait_for_navigate_to_result()
            return False
        else:
            rospy.loginfo('Riding on succeded.')

        if not self.silent_mode:
            self.sound_client.say('{}階に行きます'.format(end_floor), blocking=False)

        # start door openning check from inside
        self.subscriber_door_check = rospy.Subscriber(
            '/spot_recognition/elevator_door_points',
            PointCloud2,
            self.door_point_cloud_callback)

        # check if the door is closed
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self.door_is_open:
                break
        rospy.loginfo('elevator door closed')

        # check if the door is open and at the target floor
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.loginfo('door_is_open: {}, is_target_floor: {}, elevator stop: {}'.format(
                self.door_is_open, end_floor == self.current_floor, self.rest_elevator))
            if self.door_is_open and end_floor == self.current_floor and self.rest_elevator:
                break
        rospy.loginfo('elevator door opened and at the target_floor')

        # dance before starting to move
        self.spot_client.pub_body_pose(0.0, Quaternion(w=1))
        self.spot_client.stand()
        rospy.sleep(0.5)
        self.spot_client.pub_body_pose(-0.2, Quaternion(w=1))
        self.spot_client.stand()
        rospy.sleep(0.5)
        self.spot_client.pub_body_pose(0.0, Quaternion(w=1))
        self.spot_client.stand()

        # get off the elevator
        self.spot_client.navigate_to(end_id, blocking=True)
        result = self.spot_client.get_navigate_to_result()

        return result.success

    def run_final(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_finalize() called')

        if self.subscriber_door_check != None:
            self.subscriber_door_check.unregister()
            self.subscriber_door_check = None

        if self.subscriber_current_floor != None:
            self.subscriber_current_floor.unregister()
            self.subscriber_current_floor = None

        if self.subscriber_rest_elevator != None:
            self.subscriber_rest_elevator.unregister()
            self.subscriber_rest_elevator = None

        self.roslaunch_parent.shutdown()
