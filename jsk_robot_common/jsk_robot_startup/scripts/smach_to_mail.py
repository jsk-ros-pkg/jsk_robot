#!/usr/bin/env python

from __future__ import print_function

import actionlib
import base64
import cv2
import datetime
import os
import pickle
import rospy
import sys

from cv_bridge import CvBridge
from jsk_robot_startup.msg import Email
from jsk_robot_startup.msg import EmailBody
from sensor_msgs.msg import CompressedImage
from smach_msgs.msg import SmachContainerStatus
from std_msgs.msg import String

_enable_google_chat = False
try:
    from google_chat_ros.msg import Card
    from google_chat_ros.msg import Section
    from google_chat_ros.msg import SendMessageAction
    from google_chat_ros.msg import SendMessageActionGoal
    from google_chat_ros.msg import WidgetMarkup
    _enable_google_chat = True
except ImportError as e:
    print('Google chat ROS is not installed.', file=sys.stderr) 
    print('Disable Google chat ROS', file=sys.stderr)


class SmachToMail():

    def __init__(self):
        rospy.init_node('server_name')
        # it should be 'smach_to_mail', but 'server_name'
        # is the default name of smach_ros
        self.use_mail = rospy.get_param("~use_mail", True)
        self.use_twitter = rospy.get_param("~use_twitter", True)
        self.use_google_chat = rospy.get_param(
            "~use_google_chat", _enable_google_chat)
        self.pub_email = rospy.Publisher("email", Email, queue_size=10)
        self.pub_twitter = rospy.Publisher("tweet", String, queue_size=10)
        rospy.Subscriber(
            "~smach/container_status", SmachContainerStatus, self._status_cb)
        self.bridge = CvBridge()
        self.smach_state_list = {}  # for status list
        self.smach_state_subject = {}  # for status subject
        if rospy.has_param("~sender_address"):
            self.sender_address = rospy.get_param("~sender_address")
        else:
            rospy.logerr("Please set rosparam {}/sender_address".format(
                rospy.get_name()))
        if rospy.has_param("~receiver_address"):
            self.receiver_address = rospy.get_param("~receiver_address")
        else:
            rospy.logerr("Please set rosparam {}/receiver_address".format(
                    rospy.get_name()))

        self.chat_space = rospy.get_param("~google_chat_space", None)
        if self.use_google_chat and self.chat_space is None:
            rospy.logerr("Please set rosparam ~google_chat_space")
            self.use_google_chat = False

        if self.use_google_chat:
            self.gchat_ac = actionlib.SimpleActionClient("/google_chat_ros/send", SendMessageAction)
            self.gchat_image_dir = rospy.get_param("~google_chat_tmp_image_dir", "/tmp")
            self._gchat_thread = None

    def _status_cb(self, msg):
        '''
        Recording starts when smach_state becomes start.
        '''
        rospy.loginfo("Received SMACH status")
        if len(msg.active_states) == 0:
            return

        # Print received messages
        status_str = ', '.join(msg.active_states)
        if sys.version_info.major < 3:
            local_data_str = pickle.loads(msg.local_data)
        else:
            local_data_str = pickle.loads(
                msg.local_data.encode('utf-8'), encoding='utf-8')
        info_str = msg.info
        if not type(local_data_str) is dict:
            rospy.logerr("local_data_str:({}) is not dictionary".format(local_data_str))
            rospy.logerr("May be you forget to pass user data in exec-state-machine ?")
            rospy.logerr("please check your program execute smach with (exec-state-machine (sm) '((description . "")(image . "")))")

        rospy.loginfo("- status -> {}".format(status_str))
        rospy.loginfo("- info_str -> {}".format(info_str))
        if 'DESCRIPTION' in local_data_str:
            rospy.loginfo("- description_str -> {}".format(local_data_str['DESCRIPTION']))
        else:
            rospy.logwarn("smach does not have DESCRIPTION, see https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_robot_common/jsk_robot_startup#smach_to_mailpy for more info")
        if 'IMAGE' in local_data_str and local_data_str['IMAGE']:
            rospy.loginfo("- image_str -> {}".format(local_data_str['IMAGE'][:64]))
        if 'INFO' in local_data_str:
            rospy.loginfo("- info_str -> {}".format(local_data_str['INFO']))
        else:
            rospy.logwarn("smach does not have INFO, see https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_robot_common/jsk_robot_startup#smach_to_mailpy for more info")


        # Store data for every callerid to self.smach_state_list[caller_id]
        caller_id = msg._connection_header['callerid']

        # If we received START/INIT status, restart storing smach_state_list
        if status_str in ["START", "INIT"]:
            self.smach_state_list[caller_id] = []
            # DESCRIPTION of START is MAIL SUBJECT
            if 'DESCRIPTION' in local_data_str:
                self.smach_state_subject[caller_id] = local_data_str['DESCRIPTION']
                del local_data_str['DESCRIPTION']
            else:
                self.smach_state_subject[caller_id] = None

        # Build status_dict for every status
        # expected keys are 'DESCRIPTION' , 'IMAGE', 'STATE', 'INFO', 'TIME'
        status_dict = {}
        if 'DESCRIPTION' in local_data_str:
            status_dict.update({'DESCRIPTION': local_data_str['DESCRIPTION']})

        if 'IMAGE' in local_data_str and local_data_str['IMAGE']:
            imgmsg = CompressedImage()
            imgmsg.deserialize(base64.b64decode(local_data_str['IMAGE']))
            cv_image = self.bridge.compressed_imgmsg_to_cv2(imgmsg, "bgr8")
            scale_percent = 640.0 / cv_image.shape[1] * 100.0
            width = int(cv_image.shape[1] * scale_percent / 100)
            height = int(cv_image.shape[0] * scale_percent / 100)
            dim = (width, height)
            cv_image = cv2.resize(cv_image, dim, interpolation = cv2.INTER_AREA)
            status_dict.update({'IMAGE': base64.b64encode(cv2.imencode('.jpg', cv_image)[1].tostring())})  # dict is complicated?
        status_dict.update({'STATE': status_str})
        status_dict.update({'INFO': info_str})
        status_dict.update({'TIME': datetime.datetime.fromtimestamp(msg.header.stamp.to_sec())})


        if (caller_id not in self.smach_state_list) or self.smach_state_list[caller_id] is None:
            rospy.logwarn("received {}, but we did not find START node".format(status_dict))
        else:
            self.smach_state_list[caller_id].append(status_dict)

        # If we received END/FINISH status, send email, etc...
        if status_str in ["END", "FINISH", "FINISH-SUCCESS", "FINISH-FAILURE"]:
            if (caller_id not in self.smach_state_list) or self.smach_state_list[caller_id] is None:
                rospy.logwarn("received END node, but we did not find START node")
                rospy.logwarn("failed to send {}".format(status_dict))
            else:
                rospy.loginfo("END!!")
                rospy.loginfo("Following SMACH is reported")
                for x in self.smach_state_list[caller_id]:
                    rospy.loginfo(" - At {}, Active state is {}{}".format(x['TIME'], x['STATE'],
                                  "({})".format(x['INFO']) if x['INFO'] else ''))
                if self.use_mail:
                    self._send_mail(self.smach_state_subject[caller_id], self.smach_state_list[caller_id])
                if self.use_twitter:
                    self._send_twitter(self.smach_state_subject[caller_id], self.smach_state_list[caller_id])
                if self.use_google_chat:
                    self._send_google_chat(self.smach_state_subject[caller_id], self.smach_state_list[caller_id])
                self.smach_state_list[caller_id] = None

    def _send_mail(self, subject, state_list):
        email_msg = Email()
        email_msg.body = []
        changeline = EmailBody()
        changeline.type = 'html'
        changeline.message = "<br>"
        separator = EmailBody()
        separator.type = 'text'
        separator.message = "---------------"
        for x in state_list:
            if 'DESCRIPTION' in x:
                description = EmailBody()
                description.type = 'text'
                description.message = x['DESCRIPTION']
                email_msg.body.append(description)
                email_msg.body.append(changeline)
            if 'IMAGE' in x and x['IMAGE']:
                image = EmailBody()
                image.type = 'img'
                image.img_size = 100
                image.img_data = x['IMAGE']
                email_msg.body.append(image)
                email_msg.body.append(changeline)
        email_msg.body.append(changeline)
        email_msg.body.append(changeline)
        email_msg.body.append(separator)
        email_msg.body.append(changeline)
        for x in state_list:
            if 'INFO' in x:
                info = EmailBody()
                info.type = 'text'
                info.message = x['INFO']
                email_msg.body.append(info)
                email_msg.body.append(changeline)
        # rospy.loginfo("body:{}".format(email_msg.body))

        if subject:
            email_msg.subject = subject
        else:
            email_msg.subject = 'Message from {}'.format(rospy.get_param('/robot/name', 'robot'))

        email_msg.sender_address = self.sender_address
        email_msg.receiver_address = self.receiver_address

        rospy.loginfo("send '{}' email to {}".format(email_msg.subject, email_msg.receiver_address))

        self.pub_email.publish(email_msg)

    def _send_twitter(self, subject, state_list):
        text = u""
        if subject:
            # In python2, str is byte object, so we need to decode it as utf-8
            if isinstance(subject, bytes):
                subject = subject.decode('utf-8')
            text += subject
        prev_text_type = ''
        for x in state_list:
            if 'DESCRIPTION' in x and x['DESCRIPTION']:
                desc = x['DESCRIPTION']
                if isinstance(desc, bytes):
                    desc = desc.decode('utf-8')
                text += '\n' + desc
                prev_text_type = 'DESCRIPTION'
            if 'IMAGE' in x and x['IMAGE']:
                img_txt = x['IMAGE']
                if isinstance(img_txt, bytes):
                    img_txt = img_txt.decode('utf-8')
                if prev_text_type == 'IMAGE':
                    # [rostwitter] Do not concatenate
                    # multiple base64 images without spaces.
                    text += ' '
                text += img_txt
                prev_text_type = 'IMAGE'
        if len(text) > 1:
            self.pub_twitter.publish(String(text))

    def _send_google_chat(self, subject, state_list):
        self.gchat_ac.wait_for_server()
        goal = SendMessageActionGoal()
        if subject:
            goal.goal.text = subject
        card = Card()
        for i, x in enumerate(state_list):
            section = Section()
            widget = WidgetMarkup()
            if 'DESCRIPTION' in x:
                text = x['DESCRIPTION']
                section.header = text
            if 'IMAGE' in x and x['IMAGE']:
                path = os.path.join(self.gchat_image_dir, 'smach_gchat_{}.png'.format(i))
                with open(path, "wb") as f:
                    f.write(base64.b64decode(x['IMAGE']))
                widget.image.localpath = path
            if section.header and widget.image.localpath:
                section.widgets.append(widget)
                card.sections.append(section)
        goal.goal.cards.append(card)
        goal.goal.space = self.chat_space
        if self._gchat_thread:
            goal.goal.thread_name = self._gchat_thread
        self.gchat_ac.send_goal(goal.goal)
        self.gchat_ac.wait_for_result()
        result = self.gchat_ac.get_result()
        if not self._gchat_thread:
            self._gchat_thread = result.message_result.thread_name
        rospy.loginfo("Sending google chat messsage: {} to {} chat space".format(text, self.chat_space))
        rospy.logdebug("Google Chat result: {}".format(self.gchat_ac.get_result()))

if __name__ == '__main__':
    stm = SmachToMail()
    rospy.spin()
