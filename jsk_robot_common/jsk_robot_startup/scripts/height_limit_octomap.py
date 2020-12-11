#! /usr/bin/env python

import rospy
import tf
import std_srvs.srv
import dynamic_reconfigure.client
import threading
import time

class height_limit_octomap:
    def __init__(self):
        rospy.init_node("height_limit_octomap")
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(rospy.get_param("~rate", 0.1)) # [Hz]
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.foot_frame = rospy.get_param("~foot_frame", "base_footprint")
        self.lower_limit = rospy.get_param("~lower_limit", 0.15) # [m]
        self.upper_limit = rospy.get_param("~upper_limit", 1.6) # [m]
        # rtabmap reads parameters in private_ns on init. But rtabmap reads parameters in ns on update_parameters.

        self.octomap_server_client = dynamic_reconfigure.client.Client("octomap_server", timeout=5)

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
            (trans,rot) = self.listener.lookupTransform(self.odom_frame, self.foot_frame, rospy.Time(0))
            foot_z = trans[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("[%s] Failed to solve tf of %s to %s.", rospy.get_name(), self.odom_frame, self.foot_frame)
            return

        try:
            self.octomap_server_client.update_configuration({"occupancy_min_z":foot_z + self.lower_limit, "occupancy_max_z":foot_z + self.upper_limit})
            return
        except (rospy.ROSException, dynamic_reconfigure.DynamicReconfigureCallbackException), e:
            rospy.logwarn("[%s] update_configuration failed: %s.", rospy.get_name(), e)
            return

    def enable(self):
        rospy.loginfo("[%s] enable called.", rospy.get_name())
        if not self.is_enabled:
            config = self.octomap_server_client.get_configuration(timeout=5.0)
            if config:
                self.default_lower_limit = config["occupancy_min_z"]
                self.default_upper_limit = config["occupancy_max_z"]
            self.is_enabled = True
            self.set_limit()
        return True

    def disable(self):
        rospy.loginfo("[%s] disable called.", rospy.get_name())
        if self.is_enabled:
            try:
                self.octomap_server_client.update_configuration({"occupancy_min_z":self.default_lower_limit, "occupancy_max_z":self.default_upper_limit})
            except (rospy.ROSException, dynamic_reconfigure.DynamicReconfigureCallbackException), e:
                rospy.logwarn("[%s] update_configuration failed: %s.", rospy.get_name(), e)
            self.is_enabled = False
        return True

if __name__ == '__main__':
    node = height_limit_octomap()
    node.execute()
