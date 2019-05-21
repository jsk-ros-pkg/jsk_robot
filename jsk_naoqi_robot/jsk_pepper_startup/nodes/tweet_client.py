#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def tweet (name):
    rospy.sleep(3)
    msg = String()
    msg.data = "おはよう\n\n" + str(name)
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("pepper_twitter_client")
    pub = rospy.Publisher('/pepper_tweet', String, queue_size=10)
    name = rospy.get_param("~robot_name", "Pepper")
    tweet(name)
    exit(0)
