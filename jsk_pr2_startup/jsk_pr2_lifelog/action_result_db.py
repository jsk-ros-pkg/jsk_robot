#!/usr/bin/python
#
# This script stores All actionlib goals and results by PostgreSQL
# And store the current robot joints when recieve result.
#
# table configuration is bottom of this script
#

# parameters:
#    dbname,hostname,port,username,passwd: for DB connection
#    except action type:
#    joint_tolerance: store the past joint_states (sec)

import roslib; roslib.load_manifest('jsk_pr2_startup')
import rospy
import pgdb
import tf
import cStringIO
import thread

import std_msgs.msg
import actionlib_msgs.msg
import sensor_msgs.msg
from sensor_msgs.msg import JointState

def message_serialize(msg):
    buf = cStringIO.StringIO()
    msg.serialize(buf)
    return buf.getvalue().encode('hex_codec')

class ActionResultDB:
    loaded_types = []
    useless_types = ['roslib.msg.Header'] # message types not but action (goal|result)
    subscribers = {} # topicname:subscriber

    def __init__(self,connection=None,db_lock=None): # TODO
        # args dbname, host, port, opt, tty, user, passwd
        db_name = rospy.get_param('~db_name','pr2db')
        host = rospy.get_param('~host_name','c1')
        port = rospy.get_param('~port',5432)
        username = rospy.get_param('~user_name','pr2admin')
        passwd = rospy.get_param('~passwd','')
        self.con = pgdb.connect(database=db_name, host=host, user=username, password=passwd)
        self.lockobj = thread.allocate_lock()
        # initialize
        self.joint_tolerance = 1.0
        self.joint_states = []
        self.joint_states_inserted = [] # list of time stamp
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        return

    # callback functions
    def action_goal_cb(self, topic, type_name, msg):
        self.lockobj.acquire()
        self.insert_action_goal(topic,type_name,msg)
        self.lockobj.release()
        return

    def action_result_cb(self, topic, type_name, msg):
        self.lockobj.acquire()
        self.insert_action_result(topic,type_name,msg)
        key_func = (lambda js: abs(js.header.stamp-msg.header.stamp))
        nearest_state = min(self.joint_states,key=key_func)
        self.insert_joint_states(nearest_state)
        self.lockobj.release()
        return

    def joint_states_cb(self, msg):
        self.lockobj.acquire()
        self.joint_states = [m for m in self.joint_states if (msg.header.stamp - m.header.stamp).to_sec() < self.joint_tolerance] + [msg]
        self.joint_states_inserted = [s for s in self.joint_states_inserted if (msg.header.stamp - s).to_sec() < self.joint_tolerance]
        self.lockobj.release()
        return

    # insert functions
    def insert_action_goal(self,topic,type_name,msg):
        rospy.loginfo("insert action_goal message");
        goal_string = message_serialize(msg.goal)
        cursor = self.con.cursor()
        cursor.execute("INSERT INTO action_goal (topic,type,header_stamp, header_frame_id, goal_id_stamp, goal_id_id, goal) VALUES ('%s','%s',%d,'%s',%d,'%s','%s');" % (topic,type_name,msg.header.stamp.to_nsec(), msg.header.frame_id, msg.goal_id.stamp.to_nsec(), msg.goal_id.id, goal_string))
        cursor.close()
        self.con.commit()
        return

    def insert_action_result(self,topic,type_name,msg):
        rospy.loginfo("insert action_result message");
        result_string = message_serialize(msg.result)
        cursor = self.con.cursor()
        cursor.execute("INSERT INTO action_result (topic,type,header_stamp, header_frame_id, status_goal_id_stamp, status_goal_id_id, status_status, status_text, result) VALUES ('%s','%s',%d,'%s',%d,'%s',%d,'%s','%s');" % (topic,type_name, msg.header.stamp.to_nsec(), msg.header.frame_id, msg.status.goal_id.stamp.to_nsec(), msg.status.goal_id.id, msg.status.status, msg.status.text, result_string))
        cursor.close()
        self.con.commit()
        return

    def insert_joint_states(self,msg):
        if msg.header.stamp in self.joint_states_inserted:
            return
        rospy.loginfo("insert joint_states message");
        msg_string = message_serialize(msg)
        cursor = self.con.cursor()
        cursor.execute("INSERT INTO joint_states (header_stamp, header_frame_id, msg) VALUES (%d,'%s','%s');" % (msg.header.stamp.to_nsec(), msg.header.frame_id, msg_string))
        cursor.close()
        self.con.commit()
        return

    # if the message type is goal or result, return the callback
    def message_callback_type(self,name,type_name,type_obj):
        if not hasattr(type_obj,'header'): return None
        if type(type_obj.header) != std_msgs.msg.Header: return None
        if type_name in ['JointTrajectoryActionGoal',
                         'JointTrajectoryActionResult',
                         'PointHeadActionGoal',
                         'PointHeadActionResult',
                         'SingleJointPositionActionGoal',
                         'SingleJointPositionActionResult',
                         'Pr2GripperCommandActionGoal',
                         'Pr2GripperCommandActionResult']:
            return None
        if hasattr(type_obj,'goal_id') and hasattr(type_obj,'goal') and type(type_obj.goal_id) == actionlib_msgs.msg.GoalID:
            return (lambda msg: self.action_goal_cb(name,type_name,msg))
        if hasattr(type_obj,'status') and hasattr(type_obj,'result') and type(type_obj.status) == actionlib_msgs.msg.GoalStatus:
            return (lambda msg: self.action_result_cb(name,type_name,msg))
        return None

    # subscriber updater
    def update_subscribers(self):

        # check connections
#        for name in self.subscribers.keys():
#            sub = self.subscribers[name]
#            if sub.get_num_connections() == 0:
#                sub.unregister()
#                self.subscribers.pop(name)
#                rospy.loginfo('unsubscribe (%s)',name)

        # check new publishers
        current_subscribers = rospy.client.get_published_topics()
        for topic_info in current_subscribers:
            name = topic_info[0]
            typ_tuple  = tuple(topic_info[1].split('/'))
            typ = '%s.msg.%s'%typ_tuple
            if typ in self.useless_types:
                continue
            if name in self.subscribers.keys():
                continue

            # Import and Check
            try:
                type_obj = eval(typ)()
            except (AttributeError, NameError), e:
                try:
                    rospy.loginfo("try to load [%s]", typ_tuple[0]);
                    roslib.load_manifest(typ_tuple[0])
                    exec('import ' + typ_tuple[0] + '.msg')
                except SyntaxError, e:
                    rospy.loginfo('please rosmake %s', typ_tuple[0])

            try:
                type_class = eval(typ)
                cb_obj = self.message_callback_type(name,typ_tuple[1],type_class())
                if cb_obj == None:
                    self.useless_types += [typ]
                    continue
                self.subscribers[name] = rospy.Subscriber(name,type_class,cb_obj)
                rospy.loginfo("start subscribe (topic=%s type=%s)", name, typ);
            except Exception, e:
                self.useless_types += [typ]
                rospy.loginfo('error in checking '+typ_tuple+e)
                continue
        return

if __name__ == "__main__":
    rospy.init_node('action_result_db')
    obj = ActionResultDB()
    looprate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        obj.update_subscribers()
        looprate.sleep()

'''
CREATE TABLE action_goal(
  id               SERIAL PRIMARY KEY UNIQUE,
  topic            TEXT,
  type             TEXT,
  header_stamp     BIGINT,
  header_frame_id  TEXT,
  goal_id_stamp    BIGINT,
  goal_id_id       TEXT,
  goal             TEXT);
'''
'''
CREATE TABLE action_result(
  id                    SERIAL PRIMARY KEY UNIQUE,
  topic                 TEXT,
  type                  TEXT,
  header_stamp          BIGINT,
  header_frame_id       TEXT,
  status_goal_id_stamp  BIGINT,
  status_goal_id_id     TEXT,
  status_status         INTEGER,
  status_text           TEXT,
  result                TEXT);
'''
'''
CREATE TABLE joint_states(
  id               SERIAL PRIMARY KEY UNIQUE,
  header_stamp     BIGINT,
  header_frame_id  TEXT,
  msg              TEXT);
'''
# goal,result,msg is a serialized message object and coded in hex
