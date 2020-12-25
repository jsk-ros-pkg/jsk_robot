#!/usr/bin/env python

import rospy
import tf2_ros

import PyKDL

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdometryTransformer(object):

    def __init__(self):

        rospy.init_node('odometry_transformer')

        self._frame_id_base_link = str(rospy.get_param('~frame_id_base_link','base_link'))
        self._frame_id_pose_frame = str(rospy.get_param('~frame_id_pose_frame','t265_pose_frame'))

        self._publish_tf = bool(rospy.get_param('~publish_tf',True))

        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)

        try:
            transform_pose2base = self._tfBuffer.lookup_transform(
                    self._frame_id_pose_frame,
                    self._frame_id_base_link,
                    rospy.Time()
                    )
        except:
            rospy.logerr('hoge')

        frame_pose2base = PyKDL.Frame(
                            PyKDL.Rotation.Quaternion(
                                transform_pose2base.transform.rotation.x,
                                transform_pose2base.transform.rotation.y,
                                transform_pose2base.transform.rotation.z,
                                transform_pose2base.transform.rotation.w
                                ),
                            PyKDL,Vector(
                                transform_pose2base.transform.translation.x,
                                transform_pose2base.transform.translation.y,
                                transform_pose2base.transform.translation.z
                                )
                            )

        self._tf_br = tf2_ros.TransformBroadcaster()
        self._pub_odom = rospy.Publisher( '~odom_out', Odometry, queue_size=1 )
        self._sub_odom = rospy.Subscriber '~odom_in', Odometry, self.cb_odometry )

    def cb_odometry(self,msg):

        frame_odom2pose = PyKDL.Frame( PyKDL.Rotation.Quaternion(
                                        msg.pose.pose.orientation.x,
                                        msg.pose.pose.orientation.y,
                                        msg.pose.pose.orientation.z,
                                        msg.pose.pose.orientation.w ),
                                       PyKDL.Vector(
                                        msg.pose.pose.position.x,
                                        msg.pose.pose.position.y,
                                        msg.pose.pose.position.z )
                                       )
        frame_odom2base = frame_odom2pose * frame_pose2base

        twist_pose = PyKDL.Twist(
                        PyKDL.Vector(
                            msg.twist.twist.linear.x,
                            msg.twist.twist.linear.y,
                            msg.twist.twist.linear.z
                            ),
                        PyKDL.Vector(
                            msg.twist.twist.angular.x,
                            msg.twist.twist.angular.y,
                            msg.twist.twist.angular.z
                            )
                        )
        twist_base = frame_pose2base.Inverse() * twist_pose

        pub_msg = Odometry()
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.header.frame_id = msg.header.frame_id
        pub_msg.child_frame_id = msg.child_frame_id
        pub_msg.pose.pose.position.x = frame_odom2base.p[0]
        pub_msg.pose.pose.position.y = frame_odom2base.p[1]
        pub_msg.pose.pose.position.z = frame_odom2base.p[2]
        pub_msg.pose.pose.orientation.x = frame_odom2base.M.GetQuaternion[0]
        pub_msg.pose.pose.orientation.y = frame_odom2base.M.GetQuaternion[1]
        pub_msg.pose.pose.orientation.z = frame_odom2base.M.GetQuaternion[2]
        pub_msg.pose.pose.orientation.w = frame_odom2base.M.GetQuaternion[3]
        pub_msg.twist.twist.linear.x = twist_base.vel[0]
        pub_msg.twist.twist.linear.y = twist_base.vel[1]
        pub_msg.twist.twist.linear.z = twist_base.vel[2]
        pub_msg.twist.twist.angular.x = twist_base.rot[0]
        pub_msg.twist.twist.angular.y = twist_base.rot[1]
        pub_msg.twist.twist.angular.z = twist_base.rot[2]
        # TODO: tranform covariance
        pub_msg.pose.covariance = msg.pose.covariance
        pub_msg.twist.covariance = msg.twist.covariance
        self._pub_odom.publish( pub_msg )

        if self._publish_tf:

            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = pub_msg.header.frame_id
            t.child_frame_id = pub_msg.child_frame_id
            t.transform.translation.x = pub_msg.pose.pose.position.x
            t.transform.translation.y = pub_msg.pose.pose.position.y
            t.transform.translation.z = pub_msg.pose.pose.position.z
            t.transform.rotation.x = pub_msg.pose.pose.orientation.x
            t.transform.rotation.y = pub_msg.pose.pose.orientation.y
            t.transform.rotation.z = pub_msg.pose.pose.orientation.z
            t.transform.rotation.w = pub_msg.pose.pose.orientation.w
            self._tf_br.sendTransform(t)


def main():

    a = OdometryTransformer()
    rospy.spin()

if __name__ == '__main__':
    main()
