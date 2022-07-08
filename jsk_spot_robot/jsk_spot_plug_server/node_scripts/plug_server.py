#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import actionlib
import roslibpy
from queue import Queue
import threading
from std_msgs.msg import Bool
from jsk_spot_plug_server.msg import PlugAction, PlugResult, UnplugAction, UnplugResult


class PlugServer:

    def __init__(self):

        self.conversation_id = rospy.get_param('~conversation_id', 10)
        self.target_robot_url = rospy.get_param('~target_robot_url', 'fetch15.jsk.imi.i.u-tokyo.ac.jp')

        self.state_cable_connected = False
        self.reply_queue = Queue()

        self.subscriber = rospy.Subscriber('/spot/status/cable_connected', Bool, self.callback_connected)

        self.bridge_client = roslibpy.Ros(self.target_robot_url,9090)
        self.bridge_client.run()
        rospy.loginfo('bridge client connection: {}'.format(self.bridge_client.is_connected))

        self.bridged_publisher = roslibpy.Topic(self.bridge_client, '/dialogflow_text_input', 'dialogflow_task_executive/ConversationText')
        self.bridged_subscriber = roslibpy.Topic(self.bridge_client, '/dialogflow_text_reply', 'dialogflow_task_executive/ConversationText')
        self.bridged_subscriber.subscribe(self.callback_reply)

        self.as_plug = actionlib.SimpleActionServer('~plug', PlugAction, execute_cb=self.callback_plug, auto_start=False)
        self.as_unplug = actionlib.SimpleActionServer('~unplug', UnplugAction, execute_cb=self.callback_unplug, auto_start=False)

        self.as_plug.start()
        self.as_unplug.start()

        rospy.loginfo('Ready')

    def callback_reply(self, msg):
        if msg['conversation_id'] == self.conversation_id:
            self.reply_queue.put(msg['text'])

    def publish_text_to(self, text):
        self.bridged_publisher.publish(roslibpy.Message({'conversation_type':'text','conversation_id':self.conversation_id,'text':text}))

    def callback_connected(self, msg):
        self.state_cable_connected = msg.data

    def spin(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self.reply_queue.empty():
                reply = self.reply_queue.get()
                rospy.loginfo('reply: {}'.format(reply.encode('utf-8')))

        self.bridge_client.terminate()

    def callback_plug(self, goal):

        if self.as_unplug.is_active():
            result = PlugResult()
            result.success = False
            result.message = 'unplug action running'
            self.as_plug.set_aborted(result)
            return

        rospy.loginfo('Start plug action')

        self.publish_text_to('Spotのケーブルを挿して')

        deadline = rospy.Time.now() + goal.timeout
        rate = rospy.Rate(1)
        success = False
        rospy.loginfo('Wating for being plugged')
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.as_plug.is_preempt_requested():
                result = PlugResult()
                result.success = False
                result.message = 'canceled'
                self.as_plug.set_aborted(result)
                return
            rate.sleep()
            if self.state_cable_connected == True:
                success = True
                break

        if success:
            rospy.loginfo('Plug succeeded')
            result = PlugResult()
            result.success = success
            result.message = 'connected'
            self.as_plug.set_succeeded(result)
        else:
            rospy.loginfo('Plug Failed')
            result = PlugResult()
            result.success = success
            result.message = 'Failed'
            self.as_plug.set_aborted(result)

    def callback_unplug(self, goal):

        if self.as_plug.is_active():
            result = UnplugResult()
            result.success = False
            result.message = 'plug action running'
            self.as_unplug.set_aborted(result)
            return

        rospy.loginfo('Start unplug action')

        self.publish_text_to('Spotのケーブルを抜いて')

        deadline = rospy.Time.now() + goal.timeout
        rate = rospy.Rate(1)
        success = False
        rospy.loginfo('Wating for being unplugged')
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.as_unplug.is_preempt_requested():
                result = UnplugResult()
                result.success = False
                result.message = 'canceled'
                self.as_unplug.set_aborted(result)
                return
            rate.sleep()
            if self.state_cable_connected == True:
                success = True
                break

        if success:
            rospy.loginfo('Unplug succeeded')
            result = UnplugResult()
            result.success = success
            result.message = 'connected'
            self.as_unplug.set_succeeded(result)
        else:
            rospy.loginfo('Unplug Failed')
            result = UnplugResult()
            result.success = success
            result.message = 'Failed'
            self.as_unplug.set_aborted(result)


def main():

    rospy.init_node('plug_server')
    server = PlugServer()
    server.spin()

if __name__ == '__main__':
    main()
