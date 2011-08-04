#!/usr/bin/python
#
# Store the ObjectDetection message
#

# parameters:
#    dbname,hostname,port,username,passwd: for DB connection
#    table_name(default "tf"): DB table name

# fore example, table definition of "tf"
#                                  Table "public.tf"
#         Column          |  Type   |                    Modifiers
#-------------------------+---------+-------------------------------------------------
# id                      | integer | not null default nextval('tf_id_seq'::regclass)
# header_stamp            | bigint  |
# header_frame_id         | text    |
# child_frame_id          | text    |
# transform_translation_x | real    |
# transform_translation_y | real    |
# transform_translation_z | real    |
# transform_rotation_x    | real    |
# transform_rotation_y    | real    |
# transform_rotation_z    | real    |
# transform_rotation_w    | real    |
#Indexes:
#    "tf_stamp_idx" btree (header_stamp)

import roslib; roslib.load_manifest('jsk_pr2_startup')
import rospy
import pgdb
import tf

from geometry_msgs.msg import PoseStamped
from posedetection_msgs.msg import ObjectDetection

class ObjectDetectionDB:
    def __init__(self,connection=None,db_lock=None): # TODO
        db_name = rospy.get_param('~db_name','pr2db')
        host = rospy.get_param('~host_name','c1')
        port = rospy.get_param('~port',5432)
        username = rospy.get_param('~user_name','pr2admin')
        passwd = rospy.get_param('~passwd','')
        # args dbname, host, port, opt, tty, user, passwd
        if connection == None:
            self.con = pgdb.connect(database=db_name, host=host,
                                    user=username, password=passwd)
        else:
            self.con = connection
        self.tf_listener = tf.TransformListener()
        self.robot_frame = rospy.get_param('~base_frame_id','/base_link')
        self.table_name = rospy.get_param('~table_name','tf')
        self.subscribers = []
        return

    # DB Insertion function
    def insert_pose_to_db(self, table, stamp, source, target, pose):
        rospy.loginfo("insert ObjectDetection message");
        (trans,rot) = (pose.position, pose.orientation)
        cursor = self.con.cursor()
        cursor.execute("INSERT INTO %s (header_stamp,header_frame_id,child_frame_id,transform_translation_x, transform_translation_y, transform_translation_z, transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w) VALUES (%d,'%s','%s',%f,%f,%f,%f,%f,%f,%f);" % (table,stamp.to_nsec(),source,target,trans.x,trans.y,trans.z,rot.x,rot.y,rot.z,rot.w))
        cursor.close()
        self.con.commit()

    def objectdetection_cb(self, msg):
        try:
            self.tf_listener.waitForTransform(self.robot_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0))
            for obj in msg.objects:
                spose = PoseStamped(header=msg.header,pose=obj.pose)
                tpose = self.tf_listener.transformPose(self.robot_frame,spose)
                pose = tpose.pose

                trans = (pose.position.x, pose.position.y, pose.position.z)
                rot = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
                self.insert_pose_to_db(self.table_name, msg.header.stamp,
                                       msg.header.frame_id, obj.type, pose)
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

if __name__ == "__main__":
    rospy.init_node('pbjectdetecton_db')
    obj = ObjectDetectionDB()
    looprate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        obj.update_subscribers()
        looprate.sleep()

