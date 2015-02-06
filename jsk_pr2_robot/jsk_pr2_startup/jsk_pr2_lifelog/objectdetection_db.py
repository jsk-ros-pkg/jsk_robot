#!/usr/bin/python
#
# Store the ObjectDetection message
#

try:
    import roslib; roslib.load_manifest('jsk_pr2_startup')
except:
    pass

import rospy
import tf

from geometry_msgs.msg import PoseStamped, TransformStamped
from posedetection_msgs.msg import ObjectDetection

from mongodb_store.message_store import MessageStoreProxy

class ObjectDetectionDB(object):
    def __init__(self):
        self.db_name = rospy.get_param('~db_name','jsk_pr2_lifelog')
        self.col_name = rospy.get_param('~col_name', 'objectdetection_db')
        self.update_cycle = rospy.get_param('update_cycle', 1)
        self.map_frame = rospy.get_param('~map_frmae', 'map')
        self.robot_frame = rospy.get_param('~robot_frame','base_footprint')
        self.tf_listener = tf.TransformListener()
        self.msg_store = MessageStoreProxy(database=self.db_name, collection=self.col_name)
        rospy.loginfo("connected to %s.%s" % (self.db_name, self.col_name))
        rospy.loginfo("map->robot: %s -> %s" % (self.map_frame, self.robot_frame))

        self.subscribers = []

    # DB Insertion function
    def _insert_pose_to_db(self, map_to_robot, robot_to_obj):
        try:
            self.msg_store.insert(map_to_robot)
            self.msg_store.insert(robot_to_obj)
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

    def _update_subscribers(self):
        current_subscribers = rospy.client.get_published_topics()
        targets = [x for x in current_subscribers if x[1]=='posedetection_msgs/ObjectDetection' and ('_agg' in x[0])]
        for sub in self.subscribers:
            if sub.get_num_connections() == 0:
                sub.unregister()
                self.subscribers.remove(sub)
                rospy.loginfo('unsubscribe (%s)',sub.name)
        for topic_info in targets:
            if topic_info[0] in [x.name for x in self.subscribers]:
                continue
            sub = rospy.Subscriber(topic_info[0], ObjectDetection,
                                   self._objectdetection_cb)
            self.subscribers += [sub]
            rospy.loginfo('start subscribe (%s)',sub.name)

    def sleep_one_cycle(self):
        self._update_subscribers()
        rospy.sleep(self.update_cycle)

if __name__ == "__main__":
    rospy.init_node('objectdetecton_db')
    obj = ObjectDetectionDB()
    while not rospy.is_shutdown():
        obj.sleep_one_cycle()
