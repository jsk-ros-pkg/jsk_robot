#!/usr/bin/python

import rospy
import tf2_ros

try:
    import tf2 # groovy
except:
    import tf2_py as tf2
import geometry_msgs
from mongodb_store.message_store import MessageStoreProxy

class MoveBaseDB(object):
    def __init__(self):
        self.db_name = rospy.get_param('~db_name','jsk_pr2_lifelog')
        self.col_name = rospy.get_param('~col_name', 'move_base_db')
        self.update_cycle = rospy.get_param('~update_cycle', 1.0)
        self.map_frame = rospy.get_param('~map_frame','map')
        self.robot_frame = rospy.get_param('~robot_frame','base_link')
        self.tf_listener = tf2_ros.BufferClient("tf2_buffer_server")
        self.initialpose_pub = rospy.Publisher('/initialpose', geometry_msgs.msg.PoseWithCovarianceStamped)
        self.msg_store = MessageStoreProxy(database=self.db_name, collection=self.col_name)
        rospy.loginfo("connected to %s.%s" % (self.db_name, self.col_name))
        rospy.loginfo("map->robot: %s -> %s" % (self.map_frame, self.robot_frame))

        self.current_pose = None
        self.latest_pose = None
        self.latest_stamp = rospy.Time(0)
        self._load_latest_pose()
        self.pub_latest_pose()
        self.latest_exception = None

    def _insert_pose_to_db(self, map_to_robot):
        try:
            res = self.msg_store.insert(map_to_robot)
            rospy.loginfo("inserted %s : %s" % (res, map_to_robot))
        except Exception as e:
            rospy.logerr('failed to insert current robot pose to db: %s', e)

    def _load_latest_pose(self):
        updated = None
        try:
            updated = self.msg_store.query('geometry_msgs/TransformStamped', single=True, sort_query=[("$natural", -1)])
        except Exception as e:
            rospy.logerr('failed to load latest pose from db: %s' % e)
            if 'master has changed' in str(e):
                self._load_latest_pose()
            else:
                return
        if updated is not None:
            rospy.loginfo('found latest pose %s' % updated)
            self.current_pose = updated[0]
            self.latest_pose = updated[0]

    def _need_update_db(self, t):
        if self.current_pose is None:
            if self.latest_pose is None:
                return True
            else:
                return False
        diffthre = 0.1 + 1.0 / (rospy.Time.now() - self.latest_stamp).to_sec()
        diffpos = sum([(t.transform.translation.x - self.current_pose.transform.translation.x) ** 2,
                       (t.transform.translation.y - self.current_pose.transform.translation.y) ** 2,
                       (t.transform.translation.z - self.current_pose.transform.translation.z) ** 2]) ** 0.5
        diffrot = sum([(t.transform.rotation.x - self.current_pose.transform.rotation.x) ** 2,
                       (t.transform.rotation.y - self.current_pose.transform.rotation.y) ** 2,
                       (t.transform.rotation.z - self.current_pose.transform.rotation.z) ** 2,
                       (t.transform.rotation.w - self.current_pose.transform.rotation.w) ** 2]) ** 0.5
        rospy.loginfo("diffthre: %f, diffpos: %f, diffrot: %f" % (diffthre, diffpos, diffrot))
        if diffthre < diffpos or diffthre / 2.0 < diffrot:
            return True
        else:
            return False

    def insert_current_pose(self):
        try:
            transform = self.tf_listener.lookup_transform(self.map_frame,self.robot_frame,rospy.Time(0))
#            rospy.loginfo("current pr2 location: %s" % transform)
            if self._need_update_db(transform):
                self._insert_pose_to_db(transform)
                self.current_pose = transform
        except Exception as e:
            if not self.latest_exception or str(e) is self.latest_exception:
                rospy.logwarn('failed to get current robot pose from tf: %s' % e)
                self.latest_exception = str(e)

    def sleep_one_cycle(self):
        rospy.sleep(self.update_cycle)

    def pub_latest_pose(self):
        if(self.latest_pose is not None and self.initialpose_pub.get_num_connections() > 0):
            pub_msg = geometry_msgs.msg.PoseWithCovarianceStamped()
            pub_msg.header.stamp = rospy.Time(0)
            pub_msg.header.frame_id = '/map'
            pub_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
            pub_msg.pose.pose.position.x = self.latest_pose.transform.translation.x
            pub_msg.pose.pose.position.y = self.latest_pose.transform.translation.y
            pub_msg.pose.pose.position.z = self.latest_pose.transform.translation.z
            pub_msg.pose.pose.orientation = self.latest_pose.transform.rotation
            try:
                self.initialpose_pub.publish(pub_msg)
                self.latest_pose = None
                rospy.loginfo('published latest pose %s' % pub_msg)
            except Exception as e:
                rospy.logerr('failed to publish initial robot pose: %s' % e)

if __name__ == "__main__":
    rospy.init_node('move_base_db')
    obj = MoveBaseDB()
    while not rospy.is_shutdown():
        obj.insert_current_pose()
        obj.pub_latest_pose()
        obj.sleep_one_cycle()
