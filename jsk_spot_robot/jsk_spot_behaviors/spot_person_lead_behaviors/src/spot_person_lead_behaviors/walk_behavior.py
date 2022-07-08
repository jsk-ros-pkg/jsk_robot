# -*- coding: utf-8 -*-

from spot_behavior_manager.base_behavior import BaseBehavior

import roslaunch
import rospkg
import rospy

import threading

from std_msgs.msg import Bool

class WalkBehavior(BaseBehavior):

    def callback_visible(self, msg):

        if self.state_visible != msg.data:
            self.starttime_visibility = rospy.Time.now()
            self.duration_visibility = rospy.Duration()
            self.state_visible = msg.data
        else:
            self.duration_visibility = rospy.Time.now() - self.starttime_visibility

    def run_initial(self, start_node, end_node, edge, pre_edge ):

        rospy.logdebug('run_initial() called')

        # launch recognition launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch_path = rospkg.RosPack().get_path('spot_person_lead_behaviors') +\
                          '/launch/walk_detection.launch'
        roslaunch_cli_args = [roslaunch_path]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(roslaunch_cli_args)
        self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
                                        uuid,
                                        roslaunch_file
                                        )
        self.roslaunch_parent.start()

        # value for person checker
        self.subscriber_visible = None
        self.state_visible = False
        self.starttime_visibility = rospy.Time.now()
        self.duration_visibility = rospy.Duration(10)

        # start subscriber
        try:
            self.subscriber_visible = rospy.Subscriber(
                                        '/walk_detection_person_tracker/visible',
                                        Bool,
                                        self.callback_visible
                                        )
        except Exception as e:
            rospy.logerr('{}'.format(e))
            return False

        return True

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

        # start leading
        success = False
        rate = rospy.Rate(10)
        self.sound_client.say('ついてきてください',blocking=True)
        self.spot_client.navigate_to( end_id, blocking=False)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.spot_client.wait_for_navigate_to_result(rospy.Duration(0.1)):
                result = self.spot_client.get_navigate_to_result()
                success = result.success
                rospy.loginfo('result: {}'.format(result))
                break

            if not self.state_visible and self.duration_visibility > rospy.Duration(0.5):
                flag_speech = False
                def notify_visibility():
                    self.sound_client.say(
                        '近くに人が見えません',
                        volume=1.0,
                        blocking=True
                        )
                    flag_speech = False
                speech_thread = threading.Thread(target=notify_visibility)
                speech_thread.start()
                while not self.state_visible and self.duration_visibility > rospy.Duration(0.5):
                    rate.sleep()
                    self.spot_client.pubCmdVel(0,0,0)
                    if not flag_speech:
                        flag_speech = True
                        speech_thread = threading.Thread(target=notify_visibility)
                        speech_thread.start()
                    if not self.state_visible and self.duration_visibility > rospy.Duration(5.0):
                        self.spot_client.cancel_navigate_to()
                    if self.state_visible:
                        self.spot_client.navigate_to( end_id, blocking=False)

        # recovery on failure
        if not success:
            self.sound_client.say('失敗したので元に戻ります', blocking=True)
            self.spot_client.navigate_to( start_id, blocking=True)
            self.spot_client.wait_for_navigate_to_result()

        return success

    def run_final(self, start_node, end_node, edge, pre_edge ):

        rospy.logdebug('run_finalize() called')

        self.spot_client.cancel_navigate_to()

        if self.subscriber_visible != None:
            self.subscriber_visible.unregister()
            self.subscriber_visible = None

        self.roslaunch_parent.shutdown()
