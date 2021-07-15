# -*- coding: utf-8 -*-

from spot_behavior_manager.base_behavior import BaseBehavior

import rospy

class WalkBehavior(BaseBehavior):

    def run_initial(self, start_node, end_node, edge, pre_edge ):

        rospy.logdebug('run_initial() called')

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

        # start navigation
        success = False
        rate = rospy.Rate(10)
        self.sound_client.say('移動します',blocking=True)
        self.spot_client.navigate_to( end_id, blocking=False)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.spot_client.wait_for_navigate_to_result(rospy.Duration(0.1)):
                result = self.spot_client.get_navigate_to_result()
                success = result.success
                rospy.loginfo('result: {}'.format(result))
                break

        # recovery on failure
        if not success:
            self.sound_client.say('失敗したので元に戻ります', blocking=True)
            self.spot_client.navigate_to( start_id, blocking=True)
            self.spot_client.wait_for_navigate_to_result()

        return success

    def run_final(self, start_node, end_node, edge, pre_edge ):

        rospy.logdebug('run_finalize() called')
