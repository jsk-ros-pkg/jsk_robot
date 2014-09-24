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
### command for creating table
# con = pgdb.connect(database='pr2db', host='c1', user='pr2admin')
# cur = con.cursor()
# cur.execute("CREATE TABLE tf (id serial, header_stamp bigint, header_frame_id text, child_frame_id text, transform_translation_x real, transform_translation_y real, transform_translation_z real, transform_rotation_x real, transform_rotation_y real, transform_rotation_z real, transform_rotation_w real);")
# cur.execute("CREATE INDEX tf_stamp_idx ON tf (header_stamp);")
# cur.close()
# con.commit()

import rospy
import pgdb
import tf2_ros
try:
    import tf2                  # groovy
except:
    import tf2_py as tf2
import geometry_msgs

class MoveBaseDB:
    def __init__(self,connection=None,db_lock=None):
        db_name = rospy.get_param('~db_name','pr2db')
        host = rospy.get_param('~host_name','c1')
        port = rospy.get_param('~port',5432)
        username = rospy.get_param('~user_name','pr2admin')
        passwd = rospy.get_param('~passwd','')
        self.update_cycle = rospy.get_param('update_cycle', 1)
        # args dbname, host, port, opt, tty, user, passwd
        self.con = pgdb.connect(database=db_name, host=host, user=username, password=passwd)
        self.map_frame = rospy.get_param('~map_frame','map')
        self.robot_frame = rospy.get_param('~robot_frame','base_link')
        self.tf_listener = tf2_ros.BufferClient("tf2_buffer_server")
        self.initialpose_pub = rospy.Publisher('/initialpose', geometry_msgs.msg.PoseWithCovarianceStamped)
        self.current_pose = None
        self.latest_pose = None
        self.latest_stamp = rospy.Time(0)
        self.load_latest_pose()
        self.pub_latest_pose()
        return

    # DB Insertion function
    def insert_pose_to_db(self, table, stamp, source, target, pose):
        ##rospy.loginfo("insert robot_base pose"+str(pose));
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
            rospy.loginfo("latest position:  pos: %f %f %f, rot: %f %f %f %f" % (result[0][0], result[0][1], result[0][2], result[0][3], result[0][4], result[0][5], result[0][6],))
            self.current_pose = (result[0][0:3],result[0][3:])
            self.latest_pose = (result[0][0:3],result[0][3:])
        return

    def insert_current_pose(self):
        try:
            transform = self.tf_listener.lookup_transform(self.map_frame,self.robot_frame,rospy.Time(0))
            trans = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
            rot = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
            stamp = rospy.Time.now()
            pose = (list(trans),list(rot))

            diffthre = 0.1 + 1.0 / (stamp - self.latest_stamp).to_sec()
            if (self.current_pose == None
                or diffthre < (sum([(trans[i]-self.current_pose[0][i])**2 for i in [0,1,2]]) ** 0.5)
                or diffthre/2 < (sum([(rot[i]-self.current_pose[1][i])**2 for i in [0,1,2,3]]) ** 0.5)):
                self.insert_pose_to_db("tf",stamp,self.map_frame,self.robot_frame,pose)
                self.current_pose = (trans,rot)
        except (tf2.LookupException, tf2.ConnectivityException, \
                tf2.ExtrapolationException, tf2.TimeoutException, \
                tf2.TransformException):
            return

    def sleep_one_cycle(self):
        self.pub_latest_pose()
        rospy.sleep(self.update_cycle);

    def pub_latest_pose(self):
        if (self.latest_pose != None \
            and self.initialpose_pub.get_num_connections() > 0):
            try:
                transform = self.tf_listener.lookup_transform(self.map_frame, 'map', rospy.Time(0))
                trans = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
                rot = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
                (ctrans, crot) = self.latest_pose

                rospy.loginfo("set pos: %f %f %f, rot: %f %f %f %f" % (trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]))
                rospy.loginfo("    pos: %f %f %f, rot: %f %f %f %f" % (ctrans[0], ctrans[1], ctrans[2], crot[0], crot[1], crot[2], crot[3]))

                ps = geometry_msgs.msg.PoseWithCovarianceStamped()
                ps.header.stamp = rospy.Time(0)
                ps.header.frame_id = '/map'
                ps.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
                ps.pose.pose.position.x = ctrans[0] - trans[0]
                ps.pose.pose.position.y = ctrans[1] - trans[1]
                ps.pose.pose.position.z = ctrans[2] - trans[2]
                ps.pose.pose.orientation.x = crot[0]
                ps.pose.pose.orientation.y = crot[1]
                ps.pose.pose.orientation.z = crot[2]
                ps.pose.pose.orientation.w = crot[3]
                self.initialpose_pub.publish (ps);
                self.latest_pose = None
            except (tf2.LookupException, tf2.ConnectivityException, \
                    tf2.ExtrapolationException, tf2.TimeoutException, \
                    tf2.TransformException):
                return

if __name__ == "__main__":
    rospy.init_node('move_base_db')
    obj = MoveBaseDB()
    while not rospy.is_shutdown():
        obj.insert_current_pose();
        obj.sleep_one_cycle();
    exit(1)
