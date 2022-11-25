#!/usr/bin/env python

import rospy
import message_filters
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import CompressedImage, JointState
from geometry_msgs.msg import WrenchStamped, PoseStamped
from audio_common_msgs.msg import AudioData

def callback(*msgs):
    for pub, msg in zip(pubs, msgs):
        pub.publish(msg)
    

subs = [
    message_filters.Subscriber('/right_camera/image_raw/compressed', CompressedImage),
    message_filters.Subscriber('/head_camera/rgb/image_raw/compressed', CompressedImage),
    message_filters.Subscriber('/dual_panda/joint_states', JointState),
    message_filters.Subscriber('/dual_panda/rarm_ee_frame', PoseStamped),
    message_filters.Subscriber('/dual_panda/larm_ee_frame', PoseStamped),
    message_filters.Subscriber('/dual_panda/rarm_state_controller/F_ext', WrenchStamped),
    message_filters.Subscriber('/audio', AudioData),
    message_filters.Subscriber('/dual_panda/rarm_cartesian_impedance_controller/equilibrium_pose', PoseStamped),
    message_filters.Subscriber('/dual_panda/larm_cartesian_impedance_controller/equilibrium_pose', PoseStamped),
]
pubs = [
    rospy.Publisher('/synced/right_camera/image_raw/compressed', CompressedImage, queue_size=10),
    rospy.Publisher('/synced/head_camera/rgb/image_raw/compressed', CompressedImage, queue_size=10),
    rospy.Publisher('/synced/dual_panda/joint_states', JointState, queue_size=10),
    rospy.Publisher('/synced/dual_panda/rarm_ee_frame', PoseStamped, queue_size=10),
    rospy.Publisher('/synced/dual_panda/larm_ee_frame', PoseStamped, queue_size=10),
    rospy.Publisher('/synced/dual_panda/rarm_state_controller/F_ext', WrenchStamped, queue_size=10),
    rospy.Publisher('/synced/audio', AudioData, queue_size=10),
    rospy.Publisher('/synced/dual_panda/rarm_cartesian_impedance_controller/equilibrium_pose', PoseStamped, queue_size=10),
    rospy.Publisher('/synced/dual_panda/larm_cartesian_impedance_controller/equilibrium_pose', PoseStamped, queue_size=10),
]

rospy.init_node("record_topic_synchronizer")
ts = message_filters.ApproximateTimeSynchronizer(subs, 100, 0.1, allow_headerless=True)
ts.registerCallback(callback)
rospy.loginfo("Starting topics synchronizing node...")
rospy.spin()
rospy.loginfo("Finishing topics synchronizing node...")
exit()
