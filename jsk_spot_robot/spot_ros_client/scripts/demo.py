#!/usr/bin/env python

import rospy
import math
from spot_ros_client.libspotros import SpotRosClient
from geometry_msgs.msg import Quaternion # for body pose

rospy.init_node('spot_ros_client_demo')
client = SpotRosClient()

# claim
rospy.loginfo('claim')
client.claim()
rospy.sleep(2)

# power on
rospy.loginfo('power on')
client.power_on()
rospy.sleep(2)

# stand
rospy.loginfo('stand')
client.stand()
rospy.sleep(2)

# sit
rospy.loginfo('sit')
client.sit()
rospy.sleep(2)

# stand
rospy.loginfo('stand')
client.stand()
rospy.sleep(2)

# send velocity commands
rospy.loginfo('sending cmd_vel')
rate = rospy.Rate(10)
start_time = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < start_time + rospy.Duration(10):
    rate.sleep()
    client.pubCmdVel(0, 0, 0.5)

# change body pose
quaternion_rotated = Quaternion(x=0,y=0,z=math.sin(0.2),w=math.cos(0.2))
quaternion_equal = Quaternion(x=0,y=0,z=0,w=1)

rospy.loginfo('rotating body pose')
client.pubBodyPose(0,quaternion_rotated)
client.stand()
rospy.sleep(5)

client.pubBodyPose(0,quaternion_equal)
client.stand()
rospy.sleep(5)

rospy.loginfo('changing body height')
client.pubBodyPose(0.2,quaternion_equal)
client.stand()
rospy.sleep(5)

client.pubBodyPose(0,quaternion_equal)
client.stand()
rospy.sleep(5)

# switch stair mode
rospy.loginfo('switching stair mode')
client.stair_mode(True)
rospy.sleep(5)
client.stair_mode(False)
rospy.sleep(5)

# send a trajectory command
rospy.loginfo('sending trajectory command')
client.trajectory(1.0,1.0,1.57,5,blocking=True)
rospy.sleep(5)

# use graphnav navigation
## assuming spot is at initial place of example.walk
rospy.loginfo('sending navigate to command')
import rospkg
upload_filepath = rospkg.RosPack().get_path('spot_autowalk_data')+'/autowalk/eng2_73b2_to_81c1_night.walk'
rospy.loginfo('upload map')
client.upload_graph(upload_filepath)
rospy.sleep(1)
rospy.loginfo('localization')
client.set_localization_fiducial()
rospy.sleep(1)
rospy.loginfo('list graph')
waypoint_ids = client.list_graph()
rospy.sleep(1)
rospy.loginfo('send command')
client.navigate_to(waypoint_ids[-1],blocking=False)
rospy.sleep(1)
rospy.loginfo('wait for result')
client.wait_navigate_to_result(rospy.Duration(30))
rospy.sleep(1)
rospy.loginfo('get a result')
result = client.get_navigate_to_result()
