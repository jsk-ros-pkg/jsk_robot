#!/usr/bin/env python

import rospy
from jsk_robot_startup.email_topic_client import EmailTopicClient


def main():

    rospy.init_node('sample_email_topic_client')
    client = EmailTopicClient()
    receiver_address = rospy.get_param('~receiver_address')
    attached_files = rospy.get_param('~attached_files')

    rospy.sleep(5)
    client.send_mail('test-mail',receiver_address,'test',attached_files=attached_files)
    rospy.loginfo('Sent a mail')

if __name__=='__main__':
    main()
