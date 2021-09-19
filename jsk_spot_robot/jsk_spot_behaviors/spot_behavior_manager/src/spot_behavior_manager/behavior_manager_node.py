# -*- coding: utf-8 -*-

import copy

import actionlib
import rospy
import roslaunch

from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient

from spot_behavior_manager.support_behavior_graph import SupportBehaviorGraph
from spot_behavior_manager.base_behavior import BaseBehavior, load_behavior_class

from std_msgs.msg import String
from spot_behavior_manager_msgs.msg import LeadPersonAction, LeadPersonFeedback, LeadPersonResult, LeadPersonActionFeedback
from spot_behavior_manager_msgs.srv import ResetCurrentNode, ResetCurrentNodeResponse


class BehaviorManagerNode(object):

    def __init__(self):

        # navigation dictonary
        raw_edges = rospy.get_param('~map/edges')
        raw_nodes = rospy.get_param('~map/nodes')
        self.graph = SupportBehaviorGraph(raw_edges, raw_nodes)
        self.current_node_id = rospy.get_param('~initial_node_id')
        self.pre_edge = None

        # get target action name list for synchronizer
        self.list_action_name_synchronizer = rospy.get_param(
            '~action_names_synchronizer', [])

        # action clients
        self.spot_client = SpotRosClient()
        self.sound_client = SoundClient(
            blocking=False,
            sound_action='/robotsound_jp',
            sound_topic='/robotsound_jp'
        )

        # publisher
        self.pub_current_node_id = rospy.Publisher(
            '~current_node_id', String, queue_size=1)

        # reset service
        self.service_reset_current_node_id = rospy.Service(
            '~reset_current_node_id',
            ResetCurrentNode,
            self.handler_reset_current_node_id
        )

        #
        roslaunch.pmon._init_signal_handlers()

        # subscribers
        self.list_behaviors_execution_actions = []
        for action_name in self.list_action_name_synchronizer:
            self.list_behaviors_execution_actions.append(
                    rospy.Subscriber('{}/feedback'.format(action_name), LeadPersonActionFeedback, self.callback_synchronizer)
                    )

        # action server
        self.server_execute_behaviors = actionlib.SimpleActionServer(
            '~execute_behaviors',
            LeadPersonAction,
            execute_cb=self.handler_execute_behaviors,
            auto_start=False
        )
        self.server_execute_behaviors.start()

        rospy.loginfo('Initialized!')

    def callback_synchronizer(self, msg):

        rospy.loginfo('Current node is updated to {}'.format(msg.feedback.current_node_id))
        self.current_node_id = msg.feedback.current_node_id
        self.pre_edge = None

    def run(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            self.pub_current_node_id.publish(String(data=self.current_node_id))

    def handler_reset_current_node_id(self, req):

        rospy.loginfo('current_node_id is reset to {}'.format(
            req.current_node_id))
        self.current_node_id = req.current_node_id
        self.pre_edge = None
        return ResetCurrentNodeResponse(success=True)

    def handler_execute_behaviors(self, goal):

        rospy.loginfo('Lead Action started. goal: {}'.format(goal))

        current_graph = copy.deepcopy(self.graph)
        while True:
            # path calculation
            path = current_graph.calcPath(
                self.current_node_id, goal.target_node_id)
            if path is None:
                rospy.logerr('No path from {} to {}'.format(
                    self.current_node_id, goal.target_node_id))
                self.sound_client.say('パスが見つかりませんでした')
                result = LeadPersonResult(
                    success=False, message='No path found')
                self.server_execute_behaviors.set_aborted(result)
                return
            else:
                # navigation of edges in the path
                self.sound_client.say('目的地に向かいます', blocking=True)
                success_navigation = True
                for edge in path:
                    rospy.loginfo('Navigating Edge {}...'.format(edge))
                    try:
                        if self.navigate_edge(edge):
                            rospy.loginfo('Edge {} succeeded.'.format(edge))
                            self.current_node_id = edge.node_id_to
                            self.server_execute_behaviors.publish_feedback(LeadPersonFeedback(current_node_id=self.current_node_id))
                            self.pre_edge = edge
                        else:
                            rospy.logwarn('Edge {} failed'.format(edge))
                            self.sound_client.say(
                                '移動に失敗しました。経路を探索し直します。', blocking=True)
                            self.server_execute_behaviors.publish_feedback(LeadPersonFeedback(current_node_id=self.current_node_id))
                            current_graph.remove_edge(
                                edge.node_id_from, edge.node_id_to)
                            success_navigation = False
                            break
                    except Exception as e:
                        rospy.logerr(
                            'Got an error while navigating edge {}: {}'.format(edge, e))
                        self.sound_client.say('エラーが発生しました', blocking=True)
                        self.pre_edge = None
                        result = LeadPersonResult(
                            success=False,
                            message='Got an error while navigating edge {}: {}'.format(edge, e))
                        self.server_execute_behaviors.set_aborted(result)
                        return

                if success_navigation:
                    break

        rospy.loginfo('Goal Reached!')
        self.sound_client.say('目的地に到着しました.', blocking=True)
        result = LeadPersonResult(success=True, message='Goal Reached.')
        self.server_execute_behaviors.set_succeeded(result)
        return

    def navigate_edge(self, edge):

        # start node id validation
        if self.current_node_id != edge.node_id_from:
            rospy.logwarn(
                'current_node_id {} does not match node_id_from of edge ({})'.format(
                    self.current_node_id,
                    edge.node_id_from)
            )
            return False

        # load behavior class
        try:
            behavior_class = load_behavior_class(edge.behavior_type)
            behavior = behavior_class(
                self.spot_client,
                self.sound_client
            )
        except Exception as e:
            rospy.logerr(
                'Failed to load and initialize behavior class: {}'.format(e))
            self.sound_client.say('行動クラスを読み込めませんでした', blocking=True)
            self.pre_edge = None
            return False

        node_from = self.graph.nodes[edge.node_id_from]
        node_to = self.graph.nodes[edge.node_id_to]

        # Exception from behavior will be caught in handler
        success_initial = behavior.run_initial(
            node_from, node_to, edge, self.pre_edge)
        if success_initial is False:
            behavior.run_final(node_from, node_to, edge, self.pre_edge)
            return False
        else:
            success_main = behavior.run_main(
                node_from, node_to, edge, self.pre_edge)
            behavior.run_final(node_from, node_to, edge, self.pre_edge)
            return success_main
