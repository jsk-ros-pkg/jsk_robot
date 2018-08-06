#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import numpy as np
import rospy
import pymongo
from transformations import TransformListener
from geometry_msgs.msg import PoseWithCovarianceStamped
from logger_base import LoggerBase


def quaternion_distance(q1, q2):
    diff_theta = 2.0 * np.arccos(
        np.abs(np.dot(q1, q2)))
    return diff_theta


def diff_pose(p1, p2):
    pos1 = np.array([p1.position.x, p1.position.y, p1.position.z])
    pos2 = np.array([p2.position.x, p2.position.y, p2.position.z])
    q1 = np.array([p1.orientation.x, p1.orientation.y,
                     p1.orientation.z, p1.orientation.w])
    q2 = np.array([p2.orientation.x, p2.orientation.y,
                     p2.orientation.z, p2.orientation.w])
    diff_pos = pos1 - pos2
    normp = np.linalg.norm(pos1 - pos2)
    diff_theta = quaternion_distance(q1, q2)
    return normp, diff_theta


class BaseTrajectoryLogger(LoggerBase):
    def __init__(self):
        LoggerBase.__init__(self)
        self.update_rate = rospy.get_param("~update_rate", 1.0)
        self.use_amcl = rospy.get_param("~use_amcl", True)
        self.persistent = rospy.get_param("~persistent", True)
        self.thre = rospy.get_param('~thre', 0.005)
        self.rthre = rospy.get_param('~rthre', np.deg2rad(1.0))

        if self.use_amcl:
            self.map_frame = rospy.get_param("/amcl/global_frame_id")
            self.robot_frame = rospy.get_param("/amcl/base_frame_id")
        else:
            self.map_frame = rospy.get_param('~map_frame','map')
            self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        if self.map_frame.startswith("/"):
            self.map_frame = self.map_frame[1:]
        if self.robot_frame.startswith("/"):
            self.robot_frame = self.robot_frame[1:]

        self.tf_listener = TransformListener(
            use_tf2=rospy.get_param("~use_tf2", True))

        self.initialpose_pub = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped,
            queue_size=1, latch=True)

        rospy.loginfo("map->robot frame: %s -> %s" % (self.map_frame, self.robot_frame))

        try:
            self.latest_pose = self.load_latest_pose()
            # publish initial pose if possible
            self.publish_initial_pose(self.latest_pose)
        except Exception as e:
            rospy.logerr(e)

        if self.use_amcl:
            self.sub_amcl = rospy.Subscriber(
                "/amcl_pose", PoseWithCovarianceStamped,
                self.amcl_cb, queue_size=1)

    def amcl_cb(self, msg):
        rospy.logdebug("amcl_cb")
        self.latest_pose = msg

    def get_pose_from_tf(self):
        try:
            transform = self.tf_listener.lookup_transform(
                self.map_frame, self.robot_frame, rospy.Time(0))
            msg = PoseWithCovarianceStamped(header=transform.header)
            msg.pose.pose.position.x = transform.transform.translation.x
            msg.pose.pose.position.y = transform.transform.translation.y
            msg.pose.pose.position.z = transform.transform.translation.z
            msg.pose.pose.orientation.x = transform.transform.rotation.x
            msg.pose.pose.orientation.y = transform.transform.rotation.y
            msg.pose.pose.orientation.z = transform.transform.rotation.z
            msg.pose.pose.orientation.w = transform.transform.rotation.w
            # NOTE: msg.pose.covariance is left blank.
            return msg
        except Exception as e:
            rospy.logerr("Failed to get current robot pose from tf: %s" % str(e))
            return None

    def load_latest_pose(self):
        try:
            rospy.loginfo("Loading last robot pose...")
            msg, meta = self.msg_store.query(
                PoseWithCovarianceStamped._type,
                single=True,
                sort_query=[("_meta.inserted_at", pymongo.DESCENDING)])
            try:
                inserted_at = meta["inserted_at"].strftime('%Y-%m-%d %H:%M:%S')
                rospy.loginfo("Found last robot pose logged %s" % inserted_at)
            except:
                rospy.loginfo("Found last robot pose without metadata")
            return msg
        except Exception as e:
            rospy.logerr('failed to load latest pose from db: %s' % e)
        return None

    def publish_initial_pose(self, msg, timeout=10):
        if not msg: return

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.initialpose_pub.get_num_connections() > 0:
                break
            rospy.loginfo("waiting /initialpose is subscribed")
            r.sleep()

        msg.header.stamp = rospy.Time(0)
        self.initialpose_pub.publish(msg)
        rospy.loginfo("Published initial pose")

    def run(self):
        r = rospy.Rate(self.update_rate)
        prev_pose = None
        meta = dict()
        if self.persistent:
            meta.update({"persistent": True})
        thre = self.thre
        rthre = self.rthre
        while not rospy.is_shutdown():
            if not self.use_amcl:
                rospy.logdebug("Getting latest pose from tf")
                self.latest_pose = self.get_pose_from_tf()
            if self.latest_pose:
                if prev_pose:
                    dt = (rospy.Time.now() - prev_pose.header.stamp).to_sec()
                    if dt > 0:
                        diffp, diffr = diff_pose(
                            prev_pose.pose.pose, self.latest_pose.pose.pose)
                        rospy.logdebug(
                            "thre: %.2f[m], rthre: %.2f[rad], "
                            "diffpos: %.2f[m], diffrot: %.2f[rad]"
                            % (thre, rthre, diffp, diffr))
                        if thre < diffp or rthre < diffr:
                            self.insert(self.latest_pose, meta=meta)
                            prev_pose = self.latest_pose
                            rospy.loginfo("Inserted latest pose")
                else:
                    self.insert(self.latest_pose, meta=meta)
                    prev_pose = self.latest_pose
                    rospy.loginfo("Inserted latest pose")

                self.latest_pose = None

            self.spinOnce()
            r.sleep()


if __name__ == "__main__":
    rospy.init_node('base_trajectory_logger')
    BaseTrajectoryLogger().run()
