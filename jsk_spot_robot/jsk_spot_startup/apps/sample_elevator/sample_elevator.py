#!/usr/bin/env python

import rospy
from spot_ros_client.libspotros import SpotRosClient
import actionlib
from jsk_spot_behavior_msgs.msg import NavigationAction
from jsk_spot_behavior_msgs.msg import NavigationActionGoal


if __name__ == '__main__':

    rospy.init_node("sample_elevator")
    client = SpotRosClient()

    behavior_client = actionlib.SimpleActionClient("/spot_behavior_manager_server/execute_behaviors", NavigationAction)
    behavior_client.wait_for_server()

    client.claim()
    client.power_on()
    client.undock()

    behavior_client.send_goal_and_wait(NavigationActionGoal(target_node_id="eng2_3FElevator"))
    result = behavior_client.get_result()

    behavior_client.send_goal_and_wait(NavigationActionGoal(target_node_id="eng2_73B2_dock"))
    result = behavior_client.get_result()

    client.dock(521)
