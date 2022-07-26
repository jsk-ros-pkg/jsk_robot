#!/usr/bin/env python

import base64
import cv2
import datetime
import imghdr
import pickle
import rospy
import time

from cv_bridge import CvBridge
from jsk_robot_startup.msg import Email
from jsk_robot_startup.msg import EmailBody
from sensor_msgs.msg import CompressedImage
from smach_msgs.msg import SmachContainerStatus


class SmachToMail():

    def __init__(self):
        rospy.init_node('server_name')
        # it should be 'smach_to_mail', but 'server_name'
        # is the default name of smach_ros
        self.pub = rospy.Publisher("/email", Email, queue_size=10)
        rospy.Subscriber(
            "~smach/container_status", SmachContainerStatus, self._status_cb)
        self.bridge = CvBridge()
        self.smach_state_list = []  # for status list
        self.sender_address = "tsukamoto@jsk.imi.i.u-tokyo.ac.jp"
        self.receiver_address = "tsukamoto@jsk.imi.i.u-tokyo.ac.jp"

    def _status_cb(self, msg):
        '''
        Recording starts when smach_state becomes start.
        '''
        if len(msg.active_states) == 0:
            return
        file_path = None
        status_str = ', '.join(msg.active_states);
        local_data_str = pickle.loads(msg.local_data)
        info_str = msg.info
        print("status -> {}".format(status_str))
        print("description_str -> {}".format(local_data_str['DESCRIPTION']))

        if status_str == "START" or "INIT":
            self.smach_state_list = []
        if local_data_str['IMAGE']:
            imgmsg = CompressedImage()
            imgmsg.deserialize(base64.b64decode(local_data_str['IMAGE']))
            cv_image = self.bridge.compressed_imgmsg_to_cv2(imgmsg, "bgr8")
            rospy.logerr(
                "cv image type:{}".format(imghdr.what(None, cv_image)))  # for debugging
            dt_now = datetime.datetime.now()   # header should be used??

            file_path = "/tmp/{}_{}.jpg".format(
                status_str.lower(), dt_now.strftime('%y%m%d%H%M%S'))
            rospy.loginfo("filepath:{}".format(file_path))
            # if (next((x for x in self.smach_state_list if x["IMAGE"] == file_path), None)):
            #     rospy.loginfo("same file name!!!!")
            # else:
            #     rospy.loginfo("not same file name!!!")
            cv2.imwrite(file_path, cv_image)
            # cv2.imshow("Image", input_image)
            cv2.waitKey(2)
        else:
            file_path = ""

        status_dict = {'DESCRIPTION': local_data_str['DESCRIPTION'],
                       'IMAGE': file_path}  # dict is complicated?
        self.smach_state_list.append(status_dict)
        print(self.smach_state_list)
        print("info_str -> {}".format(info_str))

        if status_str == "END" or "FINISH":
            rospy.loginfo("END!!")
            self._send_mail()

    def _send_mail(self):
        email_msg = Email()
        email_msg.body = []
        changeline = EmailBody()
        changeline.type = 'html'
        changeline.message = "<br>"
        for x in self.smach_state_list:
            description = EmailBody()
            image = EmailBody()
            description.type = 'text'
            description.message = x['DESCRIPTION']
            email_msg.body.append(description)
            email_msg.body.append(changeline)
            if x['IMAGE']:
                image.type = 'img'
                image.img_size = 50
                image.file_path = x['IMAGE']
                email_msg.body.append(image)
                email_msg.body.append(changeline)
        rospy.loginfo("body:{}".format(email_msg.body))

        email_msg.subject = 'Smach mail test'

        email_msg.sender_address = self.sender_address
        email_msg.receiver_address = self.receiver_address

        time.sleep(1)
        self.pub.publish(email_msg)


if __name__ == '__main__':
    stm = SmachToMail()
    rospy.spin()
