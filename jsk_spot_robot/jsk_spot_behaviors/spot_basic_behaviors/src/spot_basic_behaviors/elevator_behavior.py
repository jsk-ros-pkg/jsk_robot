# -*- coding: utf-8 -*-

from spot_behavior_manager.base_behavior import BaseBehavior

import actionlib
import roslaunch
import rospkg
import rospy

from switchbot_ros.msg import SwitchBotCommandGoal, SwitchBotCommandAction
from sensor_msgs.msg import PointCloud2

class ElevatorBehavior(BaseBehavior):

    def door_point_cloud_callback(self, msg):
        if len(msg.data) == 0:
            self.door_is_open = True
        else:
            self.door_is_open = False

    def run_initial(self, start_node, end_node, edge, pre_edge ):

        rospy.logdebug('run_initial() called')

        # launch recognition launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch_path = rospkg.RosPack().get_path('spot_basic_behaviors') +\
                          '/launch/elevator_detection.launch'
        roslaunch_cli_args = [roslaunch_path]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(roslaunch_cli_args)
        self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
                                        uuid,
                                        roslaunch_file
                                        )
        self.roslaunch_parent.start()

        # value for door openring checker
        self.door_is_open = False
        self.subscriber_door_check = None

        # value for switchbot
        self.action_client_switchbot = actionlib.SimpleActionClient(
                                        '/switchbot_ros/switch',
                                        SwitchBotCommandAction
                                        )

    def run_main(self, start_node, end_node, edge, pre_edge ):

        rospy.logdebug('run_main() called')

        graph_name = edge.properties['graph']
        start_id = filter(
                    lambda x: x['graph'] == graph_name,
                    start_node.properties['waypoints_on_graph']
                    )[0]['id']
        end_id = filter(
                    lambda x: x['graph'] == graph_name,
                    end_node.properties['waypoints_on_graph']
                    )[0]['id']
        rest_waypoint_id = edge.properties['rest_waypoint_id']
        localization_method = filter(
                    lambda x: x['graph'] == graph_name,
                    start_node.properties['waypoints_on_graph']
                    )[0]['localization_method']

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
                ret = (False,'Unknown localization method')
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
        if not self.action_client_switchbot.wait_for_server(rospy.Duration(10)):
            rospy.logerr('switchbot server seems to fail.')
            return False
        else:
            switchbot_goal = SwitchBotCommandGoal()
            switchbot_goal.device_name = start_node.properties['switchbot_device']
            switchbot_goal.command = 'press'
            self.action_client_switchbot.send_goal(switchbot_goal)
            self.action_client_switchbot.wait_for_result()
            result = self.action_client_switchbot.get_result()
            rospy.loginfo('switchbot result: {}'.format(result))
            if not result.done:
                rospy.logerr('switchbot calling failed.')
                return False
        rospy.loginfo('elevator calling when riding on has succeeded')

        # wait for elevator
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.door_is_open:
                break
        rospy.loginfo('elevator door opened.')
        self.subscriber_door_check.unregister()
        self.subscriber_door_check = None

        # start navigation to rest point
        rate = rospy.Rate(10)
        self.spot_client.navigate_to( rest_waypoint_id, blocking=True)
        result = self.spot_client.get_navigate_to_result()
        ## recovery when riding on
        if not result.success:
            rospy.logwarn('Navigation failed when riding on')
            self.spot_client.navigate_to( start_id, blocking=True)
            self.spot_client.wait_for_navigate_to_result()
            return result.success             

        # call elevator from destination floor
        rospy.loginfo('calling elevator when getting off...')
        if not self.action_client_switchbot.wait_for_server(rospy.Duration(10)):
            rospy.logerr('switchbot server seems to fail.')
            return False
        else:
            switchbot_goal = SwitchBotCommandGoal()
            switchbot_goal.device_name = end_node.properties['switchbot_device']
            switchbot_goal.command = 'press'
            self.action_client_switchbot.send_goal(switchbot_goal)
            self.action_client_switchbot.wait_for_result()
            result = self.action_client_switchbot.get_result()
            rospy.loginfo('switchbot result: {}'.format(result))
            if not result.done:
                rospy.logerr('switchbot calling failed.')
                return False
        rospy.loginfo('elevator calling when getting off has succeeded')

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

        # check if the door is open
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.door_is_open:
                break
        rospy.loginfo('elevator door opened')

        # get off the elevator
        self.spot_client.navigate_to(end_id, blocking=True)
        result = self.spot_client.get_navigate_to_result()

        return result.success

    def run_final(self, start_node, end_node, edge, pre_edge ):

        rospy.logdebug('run_finalize() called')

        if self.subscriber_door_check != None:
            self.subscriber_door_check.unregister()
            self.subscriber_door_check = None
        self.roslaunch_parent.shutdown()
