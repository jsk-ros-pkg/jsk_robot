#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from geometry_msgs.msg import Transform, TransformStamped
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point, PointStamped
from geometry_msgs.msg import Quaternion, QuaternionStamped
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import PointCloud
import tf
import tf.transformations as T
import tf2_geometry_msgs  # NOQA  # for stamped message conversion
import tf2_ros
import numpy as np
from copy import deepcopy


class TFError(Exception):
    pass


class TransformationUtils():
    @classmethod
    def transformFromPose(cls, p):
        t = Transform()
        t.translation.x = p.position.x
        t.translation.y = p.position.y
        t.translation.z = p.position.z
        t.rotation = deepcopy(p.orientation)
        return t

    @classmethod
    def poseFromTransform(cls, t):
        p = Pose()
        p.position.x = t.translation.x
        p.position.y = t.translation.y
        p.position.z = t.translation.z
        p.orientation = deepcopy(t.rotation)
        return p

    @classmethod
    def matrixFromPosition(cls, pos):
        return T.translation_matrix((pos.x, pos.y, pos.z))

    @classmethod
    def matrixFromRotation(cls, rot):
        return T.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))

    @classmethod
    def matrixFromPose(cls, pose):
        return np.dot(cls.matrixFromPosition(pose.position), cls.matrixFromRotation(pose.orientation))

    @classmethod
    def matrixFromTransform(cls, t):
        return np.dot(cls.matrixFromPosition(t.translation), cls.matrixFromRotation(t.rotation))

    @classmethod
    def pointFromMatrix(cls, m):
        return Point(*T.translation_from_matrix(m))

    @classmethod
    def quaternionFromMatrix(cls, m):
        return Quaternion(*T.quaternion_from_matrix(m))

    @classmethod
    def transformPoseWithTransformStamped(cls, p, ts):
        m = np.dot(cls.matrixFromTransform(ts.transform), cls.matrixFromPose(p))
        pos = cls.pointFromMatrix(m)
        quat = cls.quaternionFromMatrix(m)
        ps = PoseStamped()
        ps.header.frame_id = ts.header.frame_id
        ps.header.stamp = ts.header.stamp
        ps.pose.position = pos
        ps.pose.orientation = quat
        return ps


class TransformListener(object):
    def __init__(self, use_tf2=True):
        if use_tf2:
            try:
                self.tf_listener = tf2_ros.BufferClient("/tf2_buffer_server")
                ok = self.tf_listener.wait_for_server(rospy.Duration(10))
                if not ok:
                    raise Exception("timed out: wait_for_server for 10.0 seconds")
            except Exception as e:
                rospy.logerr("Failed to initialize tf2 client: %s" % str(e))
                rospy.logwarn("Fallback to tf client")
                use_tf2 = False
        if not use_tf2:
            self.tf_listener = tf.TransformListener()
        self.use_tf2 = use_tf2

    def _wait_for_transform_tf1(self, target_frame, source_frame, time, timeout):
        try:
            self.tf_listener.waitForTransform(target_frame, source_frame, time, timeout)
            return True
        except Exception as e:
            raise TFError(e)

    def _wait_for_transform_tf2(self, target_frame, source_frame, time, timeout):
        try:
            ret = self.tf_listener.can_transform(target_frame, source_frame, time, timeout, True)
            if ret[0] > 0:
                return True
            else:
                raise Exception(ret[1])
        except Exception as e:
            raise TFError(e)

    def wait_for_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0)):
        if self.use_tf2:
            ret = self._wait_for_transform_tf2(target_frame, source_frame, time, timeout)
        else:
            ret = self._wait_for_transform_tf1(target_frame, source_frame, time, timeout)
        return ret

    def _lookup_transform_tf1(self, target_frame, source_frame, time, timeout):
        self._wait_for_transform_tf1(target_frame, source_frame, time, timeout)
        try:
            res = self.tf_listener.lookupTransform(target_frame, source_frame, time)
            if time.is_zero():
                time = self.tf_listener.getLatestCommonTime(target_frame, source_frame)
        except Exception as e:
            raise TFError(e)
        ret = TransformStamped()
        ret.header.frame_id = target_frame
        ret.header.stamp = time
        ret.child_frame_id = source_frame
        ret.transform.translation.x = res[0][0]
        ret.transform.translation.y = res[0][1]
        ret.transform.translation.z = res[0][2]
        ret.transform.rotation.x = res[1][0]
        ret.transform.rotation.y = res[1][1]
        ret.transform.rotation.z = res[1][2]
        ret.transform.rotation.w = res[1][3]
        return ret

    def _lookup_transform_tf2(self, target_frame, source_frame, time, timeout):
        try:
            return self.tf_listener.lookup_transform(target_frame, source_frame, time, timeout)
        except Exception as e:
            raise TFError(e)

    def lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0)):
        if self.use_tf2:
            ret = self._lookup_transform_tf2(target_frame, source_frame, time, timeout)
        else:
            ret = self._lookup_transform_tf1(target_frame, source_frame, time, timeout)
        return ret

    def _transform_tf1(self, msg, target_frame, timeout):
        transform_funcs = {PoseStamped: self.tf_listener.transformPose,
                           PointStamped: self.tf_listener.transformPoint,
                           PointCloud: self.tf_listener.transformPointCloud,
                           QuaternionStamped: self.tf_listener.transformQuaternion,
                           Vector3Stamped: self.tf_listener.transformVector3}
        try:
            transform = transform_funcs[type(msg)]
        except:
            raise TFError("Unsupported type: %s" % type(msg)._type)

        self._wait_for_transform_tf1(target_frame, msg.header.frame_id,
                                     msg.header.stamp, timeout)
        try:
            t = transform(target_frame, msg)
            if t.header.stamp.is_zero():
                t.header.stamp = self.tf_listener.getLatestCommonTime(target_frame, t.header.frame_id)
            return t
        except Exception as e:
            raise TFError(e)

    def _transform_tf2(self, msg, target_frame, timeout):
        try:
            transformed = self.tf_listener.transform(msg, target_frame, timeout)
            return transformed
        except tf2_ros.buffer_interface.TypeException as e:
            raise TFError(e.errstr)
        except Exception as e:
            raise TFError(e)

    def transform(self, msg, target_frame, timeout=rospy.Duration(0)):
        """
        Transform stamped object to target_frame
        `msg` must be PointStamped, PointCloud, PoseStamped, QuaternionStamped or Vector3Stamped
        """
        assert isinstance(target_frame, str)
        if self.use_tf2:
            ret = self._transform_tf2(msg, target_frame, timeout)
        else:
            ret = self._transform_tf1(msg, target_frame, timeout)
        return ret
