#!/usr/bin/env python

import rospy
import tf2_ros
from unitree_legged_msgs.msg import HighState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class UnitreeStatePublisher(object):

    def __init__(self):

        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=1)

        self.sub = rospy.Subscriber('/high_state', HighState, self.callback)

    def callback(self, msg):

        msg_odom = Odometry()
        msg_odom.header.stamp = rospy.Time.now()
        msg_odom.header.frame_id = self.odom_frame_id
        msg_odom.child_frame_id = self.base_frame_id
        msg_odom.pose.pose.position.x = msg.position[0]
        msg_odom.pose.pose.position.y = msg.position[1]
        msg_odom.pose.pose.position.z = msg.position[2]
        msg_odom.pose.pose.orientation.x = msg.imu.quaternion[0]
        msg_odom.pose.pose.orientation.y = msg.imu.quaternion[1]
        msg_odom.pose.pose.orientation.z = msg.imu.quaternion[2]
        msg_odom.pose.pose.orientation.w = msg.imu.quaternion[3]
        msg_odom.twist.twist.linear.x = msg.velocity[0]
        msg_odom.twist.twist.linear.y = msg.velocity[1]
        msg_odom.twist.twist.linear.z = msg.velocity[2]
        msg_odom.twist.twist.angular.x = msg.imu.gyroscope[0]
        msg_odom.twist.twist.angular.y = msg.imu.gyroscope[1]
        msg_odom.twist.twist.angular.z = msg.imu.gyroscope[2]
        self.pub.publish(msg_odom)

        msg_transform = TransformStamped()
        msg_transform.header.stamp = rospy.Time.now()
        msg_transform.header.frame_id = self.odom_frame_id
        msg_transform.child_frame_id = self.base_frame_id
        msg_transform.transform.translation.x = msg.position[0]
        msg_transform.transform.translation.y = msg.position[1]
        msg_transform.transform.translation.z = msg.position[2]
        msg_transform.transform.rotation.x = msg.imu.quaternion[0]
        msg_transform.transform.rotation.y = msg.imu.quaternion[1]
        msg_transform.transform.rotation.z = msg.imu.quaternion[2]
        msg_transform.transform.rotation.w = msg.imu.quaternion[3]
        self.tf_broadcaster.sendTransform(msg_transform)


if __name__ == '__main__':

    rospy.init_node('unitree_state_publisher')
    node = UnitreeStatePublisher()
    rospy.spin()
