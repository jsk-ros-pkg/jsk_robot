#! /usr/bin/python
import rospy
import message_filters
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import CompressedImage, JointState
from geometry_msgs.msg import WrenchStamped, PoseStamped
from franka_example_controllers.msg import ArmsTargetPose
from audio_common_msgs.msg import AudioData

def callback(*msgs):
    for pub, msg in zip(pubs, msgs):
        pub.publish(msg)
    

subs = [
    message_filters.Subscriber('/right_camera/image_raw/compressed', CompressedImage),
    message_filters.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage),
    message_filters.Subscriber('/dual_panda/joint_states', JointState),
    message_filters.Subscriber('/dual_panda/dual_arm_cartesian_pose_controller/right_frame', PoseStamped),
    message_filters.Subscriber('/dual_panda/dual_arm_cartesian_pose_controller/left_frame', PoseStamped),
    message_filters.Subscriber('/dual_panda/rarm_state_controller/F_ext', WrenchStamped),
    message_filters.Subscriber('/audio', AudioData),
]
pubs = [
    rospy.Publisher('/synced/right_camera/image_raw/compressed', CompressedImage, queue_size=10),
    rospy.Publisher('/synced/camera/rgb/image_raw/compressed', CompressedImage, queue_size=10),
    rospy.Publisher('/synced/dual_panda/joint_states', JointState, queue_size=10),
    rospy.Publisher('/synced/dual_panda/dual_arm_cartesian_pose_controller/right_frame', PoseStamped, queue_size=10),
    rospy.Publisher('/synced/dual_panda/dual_arm_cartesian_pose_controller/left_frame', PoseStamped, queue_size=10),
    rospy.Publisher('/synced/dual_panda/rarm_state_controller/F_ext', WrenchStamped, queue_size=10),
    rospy.Publisher('/synced/audio', AudioData, queue_size=10)
]

subs.append(message_filters.Subscriber('/dual_panda/dual_arm_cartesian_pose_controller/arms_target_pose', ArmsTargetPose))    
pubs.append(rospy.Publisher('/synced/dual_panda/dual_arm_cartesian_pose_controller/arms_target_pose', ArmsTargetPose, queue_size=10))

rospy.init_node("record_topic_syncronizer")
ts = message_filters.ApproximateTimeSynchronizer(subs, 100, 0.1, allow_headerless=True)
ts.registerCallback(callback)
rospy.loginfo("Starting topics syncronizing node...")
rospy.spin()
rospy.loginfo("Finishing topics syncronizing node...")
exit()
