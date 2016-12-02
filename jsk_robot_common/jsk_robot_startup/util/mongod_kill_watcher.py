#!/usr/bin/env python
# -*- coding: utf-8 -*-

# when this node is killed, kill mongod. 

import rospy
import signal
import os

db_path=None

def kill_mongod():
    pid = None
    try:
        with open(os.path.join(db_path, 'mongod.lock')) as f:
            pid = int(f.readline())
    except Exception as e:
        rospy.logfatal('cannot find mongod.lock: %s' % e)
        return
    rospy.sleep(5.0)
    try:
        os.kill(pid, signal.SIGINT)
        rospy.loginfo('killed mongod: %d' % pid)
    except Exception as e:
        rospy.logwarn('escallating to send SIGTERM to mongod')
        os.kill(pid, signal.SIGTERM)

if __name__ == '__main__':
    rospy.init_node('mongodb_killer')
    db_path = rospy.get_param('~db_path', '/var/lib/robot/mongodb_store')
    rospy.on_shutdown(kill_mongod)
    rospy.spin()
