#!/usr/bin/env python

import base64
import cv2
import datetime
import pickle
import rospy
import sys

from cv_bridge import CvBridge
from jsk_robot_startup.msg import Email
from jsk_robot_startup.msg import EmailBody
from sensor_msgs.msg import CompressedImage
from smach_msgs.msg import SmachContainerStatus
from std_msgs.msg import String


class SmachToMail():

    def __init__(self):
        rospy.init_node('server_name')
        # it should be 'smach_to_mail', but 'server_name'
        # is the default name of smach_ros
        self.pub_email = rospy.Publisher("email", Email, queue_size=10)
        self.pub_twitter = rospy.Publisher("tweet", String, queue_size=10)
        rospy.Subscriber(
            "~smach/container_status", SmachContainerStatus, self._status_cb)
        self.bridge = CvBridge()
        self.smach_state_list = {}  # for status list
        self.smach_state_subject = {}  # for status subject
        self.sender_address = "tsukamoto@jsk.imi.i.u-tokyo.ac.jp"
        self.receiver_address = "tsukamoto@jsk.imi.i.u-tokyo.ac.jp"

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
        if status_str in ["END", "FINISH"]:
            if (caller_id not in self.smach_state_list) or self.smach_state_list[caller_id] is None:
                rospy.logwarn("received END node, but we did not find START node")
                rospy.logwarn("failed to send {}".format(status_dict))
            else:
                rospy.loginfo("END!!")
                rospy.loginfo("Following SMACH is reported")
                for x in self.smach_state_list[caller_id]:
                    rospy.loginfo(" - At {}, Active state is {}{}".format(x['TIME'], x['STATE'],
                                  "({})".format(x['INFO']) if x['INFO'] else ''))
                self._send_mail(self.smach_state_subject[caller_id], self.smach_state_list[caller_id])
                self._send_twitter(self.smach_state_subject[caller_id], self.smach_state_list[caller_id])
                self.smach_state_list[caller_id] = None

    def _send_mail(self, subject, state_list):
        email_msg = Email()
        email_msg.body = []
        changeline = EmailBody()
        changeline.type = 'html'
        changeline.message = "<br>"
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
        if subject:
            self.pub_twitter.publish(String(subject))
            rospy.loginfo("send '{}' to twitter".format(subject))

        for x in state_list:
            text = ""
            if 'DESCRIPTION' in x:
                text = x['DESCRIPTION']
            if 'IMAGE' in x and x['IMAGE']:
                text += x['IMAGE']
            if len(text) > 1:
                self.pub_twitter.publish(String(text))
                rospy.loginfo("send '{}' to twitter".format(text[0:144]))


if __name__ == '__main__':
    stm = SmachToMail()
    rospy.spin()
