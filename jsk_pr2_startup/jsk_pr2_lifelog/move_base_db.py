#!/usr/bin/python
#
# This uses PostgreSQL and tf from /map_frame to /robot_frame
#

# parameters:
#    dbname,hostname,port,username,passwd: for DB connection
#    robot_frame: current robot base frame_id
#    map_frame:   map origin frmae_id

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

class MoveBaseDB:
    def __init__(self,connection=None,db_lock=None):
        db_name = rospy.get_param('~db_name','pr2db')
        host = rospy.get_param('~host_name','c1')
        port = rospy.get_param('~port',5432)
        username = rospy.get_param('~user_name','pr2admin')
        passwd = rospy.get_param('~passwd','')
        # args dbname, host, port, opt, tty, user, passwd
        self.con = pgdb.connect(database=db_name, host=host, user=username, password=passwd)
        self.map_frame = rospy.get_param('~map_frame','/map')
        self.robot_frame = rospy.get_param('~base_frame_id','/base_link')
        self.tf_listener = tf.TransformListener()
        self.current_pose = None
        self.latest_stamp = rospy.Time(0)
        self.load_latest_pose()
        return

    # DB Insertion function
    def insert_pose_to_db(self, table, stamp, source, target, pose):
        rospy.loginfo("insert robot_base pose"+str(pose));
        (trans,rot) = pose
        cursor = self.con.cursor()
        cursor.execute("INSERT INTO %s (header_stamp,header_frame_id,child_frame_id,transform_translation_x, transform_translation_y, transform_translation_z, transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w) VALUES (%d,'%s','%s',%f,%f,%f,%f,%f,%f,%f);" % (table,stamp.to_nsec(),source,target,trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]))
        cursor.close()
        self.con.commit()

    def load_latest_pose(self):
        cursor = self.con.cursor()
        pos_rot = cursor.execute("SELECT transform_translation_x, transform_translation_y, transform_translation_z, transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w FROM tf ORDER BY header_stamp DESC LIMIT 1")
        result = cursor.fetchall()
        if len(result) != 0:
            self.current_pose = (result[0][0:3],result[0][3:])
        return

    def insert_current_pose(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform(self.map_frame,self.robot_frame,rospy.Time(0))
            stamp = self.tf_listener.getLatestCommonTime(self.map_frame,self.robot_frame)
            pose = (list(trans),list(rot))

            diffthre = 0.1 + 1.0 / (stamp - self.latest_stamp).to_sec()
            if (diffthre < (sum([(trans[i]-self.current_pose[0][i])**2 for i in [0,1,2]]) ** 0.5) or diffthre/2 < (sum([(rot[i]-self.current_pose[1][i])**2 for i in [0,1,2,3]]) ** 0.5)):
                self.insert_pose_to_db("tf",stamp,self.map_frame,self.robot_frame,pose)
                self.current_pose = (trans,rot)
        except (tf.LookupException, tf.ConnectivityException, \
                tf.ExtrapolationException):
            return

if __name__ == "__main__":
    rospy.init_node('move_base_db')
    obj = MoveBaseDB()
    while not rospy.is_shutdown():
        obj.insert_current_pose();
        rospy.sleep(1)
    exit(1)
