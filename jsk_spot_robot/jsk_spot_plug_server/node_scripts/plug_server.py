#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import actionlib
import roslibpy
from queue import Queue

class PlugServer:

    def __init__(self):

        self.conversation_id = rospy.get_param('~conversation_id', 10)

        self.bridge_client = roslibpy.Ros('',9090)
        self.bridged_publisher = roslibpy.Topic(self.bridge_client, '/dialogflow_text_input', 'dialogflow_task_executive/ConversationText')
        self.bridged_subscriber = roslibpy.Topic(self.bridge_client, '/dialogflow_text_reply', 'dialogflow_task_executive/ConversationText')
        self.bridged_subscriber.subscribe(self.callback_reply)

        self.as_plug = actionlib.SimpleActionServer()
        self.as_unplug = actionlib.SimpleActionServer()

    def callback_reply(self, msg):
        if msg['conversation_id'] == self.conversation_id:
            self.reply_queue.put(msg['text'])

    def publish_text_to(self, text):
        self.bridged_publisher.publish(roslibpy.Message({'conversation_type':'chat','conversation_id':self.conversation_id,'text':text}))

    def 

    def callback_plug(self):

        self.publish_text_to('Spotのケーブルを挿して')

