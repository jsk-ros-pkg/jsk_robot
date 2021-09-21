#!/usr/bin/env python

# automatically reset slam as robot put on the ground

from jsk_robot_startup.msg import Email
import os
import rospy
import subprocess
import yaml


class EmailTopic(object):
    def __init__(self):
        address_yaml = rospy.get_param(
            '~address_yaml', "/var/lib/robot/email_topic.yaml")
        if os.path.exists(address_yaml):
            with open(address_yaml) as yaml_f:
                yaml_addresses = yaml.load(yaml_f)
                self.sender_address = yaml_addresses['sender_address']
                self.receiver_address = yaml_addresses['receiver_address']
        self.subscriber = rospy.Subscriber(
            'email', Email, self._cb, queue_size=1)

    def _cb(self, msg):
        # If sender_address or receiver_address is empty, use address in yaml
        if msg.sender_address == '':
            sender_address = self.sender_address
        else:
            sender_address = msg.sender_address
        if msg.receiver_address == '':
            receiver_address = self.receiver_address
        else:
            receiver_address = msg.receiver_address
        # Send email
        self._send_mail(
            msg.subject, msg.body, sender_address, receiver_address)

    def _send_mail(self, subject, body, sender_address, receiver_address):
        cmd = "LC_CTYPE=en_US.UTF-8 /bin/echo -e \"{}\"".format(body)
        cmd += " | /usr/bin/mail -s \"{}\" -r {} {}".format(
            subject, sender_address, receiver_address)
        exit_code = subprocess.call(cmd, shell=True)
        rospy.loginfo('Title: {}'.format(subject))
        if exit_code > 0:
            rospy.logerr(
                'Failed to send e-mail:  {} -> {}'.format(
                    sender_address, receiver_address))
            rospy.logerr("You may need to do '$ sudo apt install mailutils'")
        else:
            rospy.loginfo(
                'Succeeded to send e-mail: {} -> {}'.format(
                    sender_address, receiver_address))


if __name__ == "__main__":
    rospy.init_node("email_topic")
    app = EmailTopic()
    rospy.spin()
