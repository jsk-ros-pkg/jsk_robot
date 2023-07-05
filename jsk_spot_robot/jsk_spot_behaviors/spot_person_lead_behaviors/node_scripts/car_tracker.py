#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading

import PyKDL
import rospy
import message_filters
import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import Bool
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from jsk_recognition_msgs.msg import Label, LabelArray

def convert_msg_point_to_kdl_vector(point):
    return PyKDL.Vector(point.x,point.y,point.z)

class TrackedObject(object):

    UNKNOWN = 0
    FOUND = 1
    MOVING = 2
    LOST = 4

    def __init__(self,
                 label,
                 position,
                 observed_time
                 ):

        self.label = label
        self.state = self.FOUND
        self.position = position
        self.velocity = PyKDL.Vector()
        self.last_observed = observed_time
        self.lock = threading.Lock()

    def checkLost(self,
                  observed_time,
                  threshold_lost):

        with self.lock:
            if (observed_time - self.last_observed).to_sec() > threshold_lost:
                self.state = self.LOST
                return True
            else:
                return False

    def update(self,
               observed_position,
               observed_time,
               robot_position_fixedbased,
               threshold_velocity,
               threshold_distance):

        with self.lock:
            if observed_time < self.last_observed:
                rospy.logerr('observed time is earlier than last observed time, dropped.')
                return
            elif observed_time == self.last_observed:
                rospy.loginfo('this object is already observed')
                return

            # Update position and velocity
            if self.state == self.FOUND or self.state == self.MOVING or self.state == self.APPROCHING:
                self.velocity = (observed_position - self.position) / (observed_time - self.last_observed).to_sec()
            else:
                self.velocity = PyKDL.Vector()
            self.position = observed_position
            self.last_observed = observed_time

            # Update State
            if (self.state == self.LOST or self.state == self.UNKNOWN) or \
                (self.velocity.Norm() < threshold_velocity) or \
                ((robot_position_fixedbased - self.position).Norm() > threshold_distance):
                self.state = self.FOUND
            else:
                self.state = self.MOVING

    def getDistance(self, robot_position):
        return (self.position - robot_position).Norm()

class MultiObjectTracker(object):

    def __init__(self):

        self._frame_fixed = rospy.get_param('~frame_fixed', 'fixed_frame')
        self._frame_robot = rospy.get_param('~frame_robot', 'base_link')
        self._timeout_transform = rospy.get_param('~timeout_transform',0.05)
        slop = rospy.get_param('~slop', 0.1)

        # parameters for multi object trackers
        ## maximum number of tracking objects
        self._num_max_track = rospy.get_param('~num_max_track', 10)
        ## threshold for MOT state update
        self._thresholds_distance = rospy.get_param('~thresholds_distance', {})
        self._threshold_velocity = rospy.get_param('~threshold_move_velocity', 1.0)
        self._threshold_lost = rospy.get_param('~threshold_lost_duration', 1.0)

        # members
        self._lock_for_dict_objects = threading.RLock()
        self._dict_objects = {}

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Publisher
        ## publsher for if there is a moving object or not
        self._pub_visible = rospy.Publisher(
                '~visible',
                Bool,
                queue_size=1
                )

        # Subscriber
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_message('~input_bbox_array', BoundingBoxArray, 3)
                rospy.wait_for_message('~input_tracking_labels', LabelArray, 3)
                break
            except (rospy.ROSException, rospy.ROSInterruptException) as e:
                rospy.logerr('subscribing topic seems not to be published.')
                rospy.logerr('Error: {}'.format(e))
            rate.sleep()
        mf_sub_bbbox_array = message_filters.Subscriber('~input_bbox_array', BoundingBoxArray)
        mf_sub_tracking_labels = message_filters.Subscriber('~input_tracking_labels', LabelArray)
        ts = message_filters.ApproximateTimeSynchronizer([mf_sub_bbbox_array, mf_sub_tracking_labels], 10, slop)
        ts.registerCallback(self._cb_object)

    def _cb_object(self,
                   msg_bbox_array,
                   msg_tracking_labels):

        if msg_bbox_array.header.frame_id != self._frame_fixed:
            rospy.logerr('frame_id of bbox (which is {}) array must be the same `~frame_fixed` which is {}'.format(
                msg_bbox_array.header.frame_id,
                self._frame_fixed))
            return

        time_observed = msg_tracking_labels.header.stamp

        try:
            pykdl_transform_fixed_to_robot = tf2_geometry_msgs.transform_to_kdl(
                self._tf_buffer.lookup_transform(
                    self._frame_fixed,
                    self._frame_robot,
                    time_observed,
                    timeout=rospy.Duration(self._timeout_transform)
                )
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('lookup transform failed. {}'.format(e))
            return

        with self._lock_for_dict_objects:
            # add new object and update existing object state
            for bbox, tracking_label in zip(msg_bbox_array.boxes, msg_tracking_labels.labels):

                if bbox.label not in self._thresholds_distance.keys():
                    continue

                if tracking_label.id not in self._dict_objects:
                    if len(self._dict_objects) < self._num_max_track:
                        self._dict_objects[tracking_label.id] = \
                            TrackedObject(
                                bbox.label,
                                convert_msg_point_to_kdl_vector(bbox.pose.position),
                                time_observed
                                )
                    else:
                        rospy.logwarn('number of objects exceeds max track. dropped.')
                else:
                    self._dict_objects[tracking_label.id].update(
                                convert_msg_point_to_kdl_vector(bbox.pose.position),
                                time_observed,
                                pykdl_transform_fixed_to_robot.p,
                                self._threshold_velocity,
                                self._thresholds_distance[str(bbox.label)]
                                )
            # check if there is lost object
            to_be_removed = []
            for key in self._dict_objects:
                is_lost = self._dict_objects[key].checkLost(
                                time_observed,
                                self._threshold_lost
                                )
                if is_lost:
                    to_be_removed.append(key)
            # remove lost object from dict
            for key in to_be_removed:
                self._dict_objects.pop(key)

    def spin(self):

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():

            rate.sleep()

            with self._lock_for_dict_objects:

                exist_moving_object = False
                for key in self._dict_objects:
                    if self._dict_objects[key].state == TrackedObject.MOVING:
                        exist_moving_object = True

                self._pub_visible.publish(Bool(exist_moving_object))

def main():

    rospy.init_node('person_tracker')
    tracker = MultiObjectTracker()
    tracker.spin()

if __name__=='__main__':
    main()
