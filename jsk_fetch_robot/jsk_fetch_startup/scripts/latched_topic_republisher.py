#!/usr/bin/env python

import rospy
import rosbag

def main():

    rospy.init_node( 'latched_topic_republisher' )

    topicname = rospy.get_param('~topicname','tf_static')
    bagfilename = rospy.get_param("~file")

    with rosbag.Bag( bagfilename, 'r' ) as inputbag:
        topic, msg, t = inputbag.read_message( topicname ).next()
        packagename, typename = msg._type.split('/')
        exec('from {}.msg import {}'.format(packagename,typename))

    publisher = rospy.Publisher(topicname, typename, latch=True, queue_size=10)

    list_messages = []
    with rosbag.Bag( bagfilename, 'r' ) as inputbag:
        for topic, msg, t in inputbag.read_messages('/tf_static'):
            list_messages.append(msg)

    for message in list_messages:
        publisher.publish( message )

    rospy.loginfo( 'Republishing topic \'{}\''.format(topicname) )
    rospy.spin()

if __name__=='__main__':
    main()
