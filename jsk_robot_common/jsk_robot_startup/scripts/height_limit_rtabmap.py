#! /usr/bin/env python

import rospy
import tf
import std_srvs.srv
import threading
import time

class height_limit_rtabmap:
    def __init__(self):
        rospy.init_node("height_limit_rtabmap")
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(rospy.get_param("~rate", 0.1)) # [Hz]
        self.body_frame = rospy.get_param("~body_frame", "BODY")
        self.foot_frame = rospy.get_param("~foot_frame", "base_footprint")
        self.lower_limit = rospy.get_param("~lower_limit", 0.15) # [m]
        self.upper_limit = rospy.get_param("~upper_limit", 1.6) # [m]
        # rtabmap reads parameters in private_ns on init. But rtabmap reads parameters in ns on update_parameters.
        self.rtabmap_ns = rospy.get_param("~rtabmap_ns", "/rtabmap")

        self.is_enabled = False
        self.default_lower_limit = 0.0
        self.default_upper_limit = 0.0
        if rospy.get_param("~enable_start", True):
            time.sleep(1) ## wait for TF
            self.enable()

        self.lock = threading.Lock()
        self.enable_srv = rospy.Service('~enable', std_srvs.srv.SetBool, self.enable_cb)

    def execute(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.is_enabled:
                    self.set_limit()
            self.rate.sleep()

    def enable_cb(self, req):
        with self.lock:
            res = std_srvs.srv.SetBoolResponse()
            if req.data:
                res.success = self.enable()
            else:
                res.success = self.disable()
            return res

    def set_limit(self):
        try:
            rospy.wait_for_service(self.rtabmap_ns + "/update_parameters", 5)
        except rospy.ROSException:
            rospy.logwarn("[%s] Service %s not found.", rospy.get_name(), self.rtabmap_ns + "/update_parameters")
            return

        try:
            (trans,rot) = self.listener.lookupTransform(self.body_frame, self.foot_frame, rospy.Time(0))
            foot_z = trans[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("[%s] Failed to solve tf of %s to %s.", rospy.get_name(), self.body_frame, self.foot_frame)
            return

        rospy.set_param(self.rtabmap_ns + "/Grid/MaxGroundHeight", str(foot_z + self.lower_limit))
        rospy.set_param(self.rtabmap_ns + "/Grid/MaxObstacleHeight", str(foot_z + self.upper_limit))

        try:
            update_parameters = rospy.ServiceProxy(self.rtabmap_ns + "/update_parameters", std_srvs.srv.Empty)
            resp = update_parameters()
            return
        except rospy.ServiceException, e:
            rospy.logwarn("[%s] Service call %s failed: %s.", rospy.get_name(), self.rtabmap_ns + "/update_parameters", e)
            return

    def enable(self):
        rospy.loginfo("[%s] enable called.", rospy.get_name())
        if not self.is_enabled:
            self.default_lower_limit = rospy.get_param(self.rtabmap_ns + "/Grid/MaxGroundHeight", -1000.0)
            # if initial Grid/MaxGroundHeight == 0, Grid/MaxGroundHeight gets disabled. But if updated Grid/MaxGroundHeight == 0, Grid/MaxGroundHeight does not get disabled.
            if self.default_lower_limit == '0' or self.default_lower_limit == 0:
                self.default_lower_limit = -1000.0
            self.default_upper_limit = rospy.get_param(self.rtabmap_ns + "/Grid/MaxObstacleHeight", 1000.0)
            # if initial Grid/MaxObstacleHeight == 0, Grid/MaxObstacleHeight gets disabled. But if updated Grid/MaxObstacleHeight == 0, Grid/MaxObstacleHeight does not get disabled.
            if self.default_upper_limit == '0' or self.default_upper_limit == 0:
                self.default_upper_limit = 1000.0
            self.is_enabled = True
            self.set_limit()
        return True

    def disable(self):
        rospy.loginfo("[%s] disable called.", rospy.get_name())
        if self.is_enabled:
            rospy.set_param(self.rtabmap_ns + "/Grid/MaxGroundHeight", str(self.default_lower_limit))
            rospy.set_param(self.rtabmap_ns + "/Grid/MaxObstacleHeight", str(self.default_upper_limit))
            try:
                update_parameters = rospy.ServiceProxy(self.rtabmap_ns + "/update_parameters", std_srvs.srv.Empty)
                resp = update_parameters()
            except rospy.ServiceException, e:
                rospy.logwarn("[%s] Service call %s failed: %s.", rospy.get_name(), self.rtabmap_ns + "/update_parameters", e)

            self.is_enabled = False
        return True

if __name__ == '__main__':
    node = height_limit_rtabmap()
    node.execute()
