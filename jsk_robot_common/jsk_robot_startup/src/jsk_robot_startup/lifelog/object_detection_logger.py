#!/usr/bin/python
#
# Store the ObjectDetection message
#

import rospy
import tf
import rosgraph
from geometry_msgs.msg import PoseStamped, TransformStamped
from posedetection_msgs.msg import ObjectDetection
from .logger_base import LoggerBase

class ObjectDetectionLogger(LoggerBase):
    def __init__(self):
        LoggerBase.__init__(self)
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame','base_footprint')
        self.tf_listener = tf.TransformListener()
        rospy.loginfo("map->robot: %s -> %s" % (self.map_frame, self.robot_frame))

        self.master = rosgraph.Master("/rostopic")
        self.subscribers = []

    # DB Insertion function
    def _insert_pose_to_db(self, map_to_robot, robot_to_obj):
        try:
            self.insert(map_to_robot)
            self.insert(robot_to_obj)
            rospy.loginfo('inserted map2robot: %s, robot2obj: %s' % (map_to_robot, robot_to_obj))
        except Exception as e:
            rospy.logwarn('failed to insert to db' + e)

    def _lookup_transform(self, target_frame, source_frame, time=rospy.Time(0), timeout=rospy.Duration(0.0)):
        self.tf_listener.waitForTransform(target_frame, source_frame, time, timeout)
        res = self.tf_listener.lookupTransform(target_frame, source_frame, time)
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

    def _objectdetection_cb(self, msg):
        try:
            self.tf_listener.waitForTransform(self.robot_frame, self.map_frame, msg.header.stamp, rospy.Duration(1.0))
            map_to_robot = self._lookup_transform(self.robot_frame, self.map_frame, msg.header.stamp)
        except Exception as e:
            rospy.logwarn("failed to lookup tf: %s", e)
            return

        try:
            for obj in msg.objects:
                spose = PoseStamped(header=msg.header,pose=obj.pose)
                tpose = self.tf_listener.transformPose(self.robot_frame, spose)
                obj.pose = tpose.pose
                self._insert_pose_to_db(map_to_robot, obj)
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
                rospy.loginfo('unsubscribed (%s)',sub.name)
        for topic_name in targets:
            if topic_name in [x.name for x in self.subscribers]:
                continue
            sub = rospy.Subscriber(topic_name, ObjectDetection,
                                   self._objectdetection_cb)
            self.subscribers += [sub]
            rospy.loginfo('start subscribe (%s)',sub.name)


    def run(self):
        while not rospy.is_shutdown():
            self.update_subscribers()
            self.spinOnce()

if __name__ == "__main__":
    rospy.init_node('object_detecton_logger')
    ObjectDetectionLogger().run()
