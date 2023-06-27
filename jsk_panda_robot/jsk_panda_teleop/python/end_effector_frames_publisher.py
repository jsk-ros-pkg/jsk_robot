#!/usr/bin/env python

# This node extracts end effector frame from TF and publishes to single topic.
# This enables message_filters.ApproximateTimeSynchronizer
# to synchronize that frame and other topics.
# Synchronized data is used in @ykawamura96's learning system.
# It originally depended on https://github.com/frankaemika/franka_ros/pull/270,
# which adds a new controller publishing the frame.
# But we decided not to add a new controller,
# so we need this node to minimize changes of existing code.
# The system uses the frame on not only data collection but also execution.

import rospy
import tf
from geometry_msgs.msg import PoseStamped


class SingleArmPublisher(object):

    def __init__(self, robot_id='dual_panda', arm_name='larm'):
        self.robot_id = robot_id
        self.arm_name = arm_name
        self.current_frame_id = '{}_EE'.format(arm_name)
        frame_pub_topic = '{}/{}_ee_frame'.format(self.robot_id, self.arm_name)
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform(
            '{}_link0'.format(self.arm_name),
            self.current_frame_id,
            rospy.Time(),
            rospy.Duration(3600)
        )
        self.frame_pub = rospy.Publisher(
            frame_pub_topic, PoseStamped, queue_size=1)

    def publish_frame(self):
        current_frame = PoseStamped()
        (trans, rot) = self.tf_listener.lookupTransform(
            '{}_link0'.format(self.arm_name),
            self.current_frame_id,
            rospy.Time(0)
        )
        current_frame.pose.position.x = trans[0]
        current_frame.pose.position.y = trans[1]
        current_frame.pose.position.z = trans[2]
        current_frame.pose.orientation.x = rot[0]
        current_frame.pose.orientation.y = rot[1]
        current_frame.pose.orientation.z = rot[2]
        current_frame.pose.orientation.w = rot[3]
        current_frame.header.stamp = rospy.Time.now()
        current_frame.header.frame_id = '{}_link0'.format(self.arm_name)
        self.frame_pub.publish(current_frame)


class EndEffectorFramesPublisher(object):

    def __init__(self):
        self.larm_pub = SingleArmPublisher(arm_name='larm')
        self.rarm_pub = SingleArmPublisher(arm_name='rarm')
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.publish_frames)

    def publish_frames(self, event):
        self.larm_pub.publish_frame()
        self.rarm_pub.publish_frame()


if __name__ == '__main__':
    rospy.init_node('end_effector_frames_publisher')
    app = EndEffectorFramesPublisher()
    rospy.spin()
