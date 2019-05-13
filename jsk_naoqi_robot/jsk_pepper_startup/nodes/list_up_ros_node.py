#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from subprocess import *
from jsk_rviz_plugins.msg import OverlayText

class ListUpROSNode():
    def __init__(self):
        rospy.init_node("list_up_ros_node")
        # execute timer_callback every 10 sec
        self.timer = rospy.Timer(rospy.Duration(10), self.timer_callback)
        self.pub = rospy.Publisher("~output", OverlayText, queue_size=1)
        self.naoqi_driver_node_name = os.environ.get('ROS_NAMESPACE', '/pepper_robot')
        if self.naoqi_driver_node_name is not '/pepper_robot':
            self.naoqi_driver_node_name = self.naoqi_driver_node_name + self.naoqi_driver_node_name
        self.pose_controller_node_name = self.naoqi_driver_node_name + '/pose/pose_controller'

    def timer_callback(self, event):
        p=Popen(['rosnode','list'], stdout=PIPE)
        p.wait()
        nodelist_tmp=p.communicate()
        # nodelist_tmp: ('/pepper_1556630474468299832\n/rosout\n', None)
        nodelist=nodelist_tmp[0]
        # nodelist: '/pepper_1556630474468299832\n/rosout\n'
        nodelist=nodelist.split("\n")
        # nodelist: ['/pepper_1556630474468299832', '/rosout', '']

        # If '/pepper_robot' node is killed, need to kill jsk_pepper_startup.launch by killing another required node (see https://github.com/jsk-ros-pkg/jsk_robot/issues/1077)
        if self.naoqi_driver_node_name in nodelist:
            pass
        else:
            call(['rosnode', 'kill', self.pose_controller_node_name])

        tmp=[]
        add_camera_node = False
        for i in nodelist:
            # play_audio_stream_node is highlighted because this node needs to take care of people's privacy
            if i == '/play_audio_stream_node':
                i ="""<span style="color: red;">""" + i + """</span>"""
                tmp.append(i)
            # display camera node as "/pepper_robot/camera" because there are a lot of camera nodes
            else:
                if 'camera' in i:
                    if not add_camera_node:
                        i = "/pepper_robot/camera"
                        tmp.append(i)
                        add_camera_node = True
                # ignore ''
                elif i is not '':
                    tmp.append(i)

        text = OverlayText()
        text.left = 350
        text.top = 0
        text.width = 400
        text.height = 300
        text.fg_color.r = 25/255.0
        text.fg_color.g = 255/255.0
        text.fg_color.b = 255/255.0
        text.fg_color.a = 0.8
        text.bg_color.r = 0
        text.bg_color.g = 0
        text.bg_color.b = 0
        text.bg_color.a = 0.8
        text.text_size = 10
        text.text = "\n".join(tmp)
        self.pub.publish(text)

if __name__ == "__main__":
    list_up_node = ListUpROSNode()
    rospy.spin()
