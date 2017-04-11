#!/usr/bin/python
#
# Store the ObjectDetection message
#

import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped, TransformStamped
from posedetection_msgs.msg import ObjectDetection
from logger_base import LoggerBase
from transformations import TransformListener


class ObjectDetectionLogger(LoggerBase):
    def __init__(self):
        LoggerBase.__init__(self)
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame','base_footprint')
        rospy.loginfo("map->robot: %s -> %s" % (self.map_frame, self.robot_frame))

        self.tf_timeout = rospy.Duration(rospy.get_param("~tf_timeout", 1.0))
        use_tf2 = rospy.get_param("~use_tf2", False)
        self.tf_listener = TransformListener(use_tf2)

        self.master = rosgraph.Master("/rostopic")
        self.subscribers = []

        rospy.loginfo("%s initialized" % rospy.get_name())

    # DB Insertion function
    def __insert_pose_to_db(self, map_to_robot, robot_to_obj):
        try:
            self.insert(map_to_robot)
            self.insert(robot_to_obj)
            rospy.loginfo('inserted map2robot: %s, robot2obj: %s' % (map_to_robot, robot_to_obj))
        except Exception as e:
            rospy.logwarn('failed to insert to db' + e)

    def __objectdetection_cb(self, msg):
        try:
            map_to_robot = self.tf_listener.lookup_transform(self.robot_frame, self.map_frame,
                                                             msg.header.stamp, self.tf_timeout)
        except Exception as e:
            rospy.logwarn("failed to lookup tf: %s", e)
            return

        for obj in msg.objects:
            try:
                spose = PoseStamped(header=msg.header,pose=obj.pose)
                tpose = self.tf_listener.transform(self.robot_frame, spose, self.tf_timeout)
                obj.pose = tpose.pose
                self.__insert_pose_to_db(map_to_robot, obj)
            except Exception as e:
                rospy.logwarn("failed to object pose transform: %s", e)

    def update_subscribers(self):
        object_detection_topics = [x[0] for x in rospy.client.get_published_topics()
                                   if x[1]=='posedetection_msgs/ObjectDetection' and (not ('_agg' in x[0]))]
        _, subs, _ = self.master.getSystemState()
        targets = [x[0] for x in subs if x[0] in object_detection_topics and not rospy.get_name() in x[1]]
        for sub in self.subscribers:
            sub_nodes = [x[1] for x in subs if x[0] == sub.name]
            if (not sub_nodes) or (len(sub_nodes[0]) == 1 and (rospy.get_name() in sub_nodes[0])):
                sub.unregister()
                self.subscribers.remove(sub)
                rospy.logdebug('unsubscribed (%s)', sub.name)
        for topic_name in targets:
            if topic_name in [x.name for x in self.subscribers]:
                continue
            sub = rospy.Subscriber(topic_name, ObjectDetection,
                                   self.__objectdetection_cb)
            self.subscribers += [sub]
            rospy.logdebug('start subscribe (%s)', sub.name)

    def run(self):
        while not rospy.is_shutdown():
            self.update_subscribers()
            self.spinOnce()

if __name__ == "__main__":
    rospy.init_node('object_detecton_logger')
    ObjectDetectionLogger().run()
