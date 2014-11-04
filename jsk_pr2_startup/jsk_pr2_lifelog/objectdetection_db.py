#!/usr/bin/python
#
# Store the ObjectDetection message
#

try:
    import roslib; roslib.load_manifest('jsk_pr2_startup')
except:
    pass

import rospy
import tf2_ros

try:
    import tf2
except:
    import tf2_py as tf2

from geometry_msgs.msg import PoseStamped
from posedetection_msgs.msg import ObjectDetection

from mongodb_store.message_store import MessageStoreProxy

class ObjectDetectionDB(object):
    def __init__(self):
        self.db_name = rospy.get_param('~db_name','jsk_pr2_lifelog')
        self.col_name = rospy.get_param('~col_name', 'objectdetection_db')
        self.update_cycle = rospy.get_param('update_cycle', 1)
        self.map_frame = rospy.get_param('~map_frmae', 'map')
        self.robot_frame = rospy.get_param('~robot_frame','base_footprint')
        self.tf_listener = tf2_ros.BufferClient("tf2_buffer_server")
        self.msg_store = MessageStoreProxy(database=self.db_name, collection=self.col_name)

        self.subscribers = []

    # DB Insertion function
    def insert_pose_to_db(self, map_to_robot, robot_to_obj)
        self.msg_store.insert(map_to_robot)
        self.msg_store.insert(robot_to_obj)
    
    def objectdetection_cb(self, msg):
        try:
            self.tf_listener.wait_for_server(rospy.Duration(1.0))
            map_to_robot = self.tf_listener.lookup_transform(self.robot_frame, self.map_frame, msg.header.stamp)
            for obj in msg.objects:
                spose = PoseStamped(header=msg.header,pose=obj.pose)
                tpose = self.tf_listener.transform(spose, self.robot_frame)
#                tpose = self.tf_listener.transformPose(self.robot_frame,spose)
                obj.pose = tpose.pose
                self.insert_pose_to_db(map_to_robot, obj)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            return

    def update_subscribers(self):
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
                                   obj.objectdetection_cb)
            self.subscribers += [sub]
            rospy.loginfo('start subscribe (%s)',sub.name)

    def sleep_one_cycle(self):
        self.update_subscribers()
        rospy.sleep(self.update_cycle)

if __name__ == "__main__":
    rospy.init_node('objectdetecton_db')
    obj = ObjectDetectionDB()
    while not rospy.is_shutdown():
        obj.sleep_one_cycle()
    exit(1)
