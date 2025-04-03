#!/usr/bin/env python3

import rospy
from spot_ros_client.libspotros import SpotRosClient
import actionlib
from jsk_spot_behavior_msgs.msg import NavigationAction
from jsk_spot_behavior_msgs.msg import NavigationGoal


if __name__ == '__main__':

    rospy.init_node("sample_elevator")
    client = SpotRosClient()

    behavior_client = actionlib.SimpleActionClient("/spot_behavior_manager_server/execute_behaviors", NavigationAction)
    behavior_client.wait_for_server()

    client.claim()
    client.power_on()
    client.undock()

    client.reset_current_node('eng2_73B2_dock')
    client.execute_behaviors(target_node_id="eng2_3FElevator")
    client.wait_execute_behaviors_result()
    result = client.get_execute_behaviors_result()
    rospy.loginfo("Result forward: {} {}".format(result.success, result.message))

    client.execute_behaviors(NavigationGoal(target_node_id="eng2_73B2_dock"))
    result = client.get_execute_behaviors_result()
    rospy.loginfo("Result backward: {} {}".format(result.success, result.message))

    client.dock(521)
