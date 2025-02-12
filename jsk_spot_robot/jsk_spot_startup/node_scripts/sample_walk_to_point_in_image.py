#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Point, PointStamped
from spot_msgs.msg import WalkToObjectInImageAction, WalkToObjectInImageGoal

def cb(msg):
    rospy.loginfo('screenpoint callback x={}, y={}, distance={}'.format(msg.point.x, msg.point.y, distance))
    timeout = 15
    goal = WalkToObjectInImageGoal(
        image_source = image_source,
        center = msg.point,
        distance = distance,
        max_duration = rospy.Duration(timeout)
    )
    client.send_goal(goal)
    client.wait_for_result(timeout=rospy.Duration(timeout+5))
    if client.get_result().success:
        rospy.loginfo("succeeded")
    else:
        rospy.logerr("failed to walkt ot object")
    
if __name__ == '__main__':
    try:
        rospy.init_node('sample_walk_to_object_in_image')
        image_source = rospy.get_param('~image_source', 'hand_color')
        distance = rospy.get_param('~distance', 0.5)
        # subscribe point in screen
        rospy.loginfo('start subscribe screenpoint at {}'.format(image_source))
        rospy.Subscriber('screenpoint', PointStamped, cb)
        # action client for walk to point API
        rospy.loginfo('conncting to walk_to_point_in_image action')
        client = actionlib.SimpleActionClient('/spot/walk_to_object_in_image', WalkToObjectInImageAction)
        client.wait_for_server()
        #
        rospy.loginfo('start main loop')
        rospy.spin()
    except rospy.ROSInterruptException:
        ropsy.logerr("program interrupted before completion")
