#!/usr/bin/env python

from email.mime.application import MIMEApplication
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import errno
import getpass
from jsk_robot_startup.msg import Email
import os
import rospy
import smtplib
import socket
from socket import error as socket_error
import yaml


class EmailTopic(object):
    """
    This node sends email based on received rostopic (jsk_robot_startup/Email).
    Default values can be set by using `~email_info`

    The yaml file is like the following:
    subject: hello
    body: world
    sender_address: hoge@test.com
    receiver_address: fuga@test.com
    smtp_server: test.com
    smtp_port: 25
    attached_file: /home/user/Pictures/test.png
    """
    def __init__(self):
        self.email_info = {}
        yaml_path = rospy.get_param(
            '~email_info', "/var/lib/robot/email_info.yaml")
        if os.path.exists(yaml_path):
            with open(yaml_path) as yaml_f:
                self.email_info = yaml.load(yaml_f)
            rospy.loginfo(
                "{} is loaded as email config file.".format(yaml_path))
            rospy.loginfo(self.email_info)
        self.subscriber = rospy.Subscriber(
            'email', Email, self._cb, queue_size=1)

    def _cb(self, msg):
        send_mail_args = {}
        # Set default value for self._send_mail arguments
        send_mail_args['subject'] = ''
        send_mail_args['body'] = ''
        send_mail_args['sender_address'] = '{}@{}'.format(getpass.getuser(), socket.gethostname())
        send_mail_args['smtp_server'] = 'localhost'
        send_mail_args['smtp_port'] = 25
        send_mail_args['attached_file'] = None
        # Set args from topic field. If the field is empty, use value in yaml
        for field in ['subject', 'body', 'sender_address', 'receiver_address',
                      'smtp_server', 'smtp_port', 'attached_file']:
            if getattr(msg, field) != '':
                send_mail_args[field] = getattr(msg, field)
            else:
                if field in self.email_info:
                    send_mail_args[field] = self.email_info[field]
        # Send email
        self._send_mail(**send_mail_args)

    def _send_mail(
            self, subject, body, sender_address, receiver_address,
            smtp_server, smtp_port, attached_file):
        # Create MIME
        msg = MIMEMultipart()
        msg["Subject"] = subject
        msg["From"] = sender_address
        msg["To"] = receiver_address
        msg.attach(MIMEText(body, 'plain', 'utf-8'))
        # Attach file
        if attached_file is not None:
            if not os.path.exists(attached_file):
                rospy.logerr('File {} is not found.'.format(attached_file))
                return
            with open(attached_file, "rb") as f:
                part = MIMEApplication(
                    f.read(),
                    Name=os.path.basename(attached_file))
            part['Content-Disposition'] = 'attachment; filename="{}"'.format(
                os.path.basename(attached_file))
            msg.attach(part)
            rospy.loginfo('File {} is attached.'.format(attached_file))
        # SMTP Server
        try:
            server = smtplib.SMTP(smtp_server, smtp_port)
        except socket_error as serr:
            if serr.errno != errno.ECONNREFUSED:
                raise serr
            rospy.logerr('Failed to send email. {}'.format(serr.args))
            rospy.logerr(
                'Please check your smtp_server and smtp_port are correct')
            rospy.logerr(
                'If you use local smtp server, try $ sudo apt install postfix')
            return
        if server.has_extn('STARTTLS'):
            server.starttls()
        # Send Email
        server.sendmail(sender_address, receiver_address, msg.as_string())
        server.quit()
        rospy.loginfo('Send mail from {} to {} using {}:{}'.format(
            sender_address, receiver_address, smtp_server, smtp_port))


if __name__ == "__main__":
    rospy.init_node("email_topic")
    app = EmailTopic()
    rospy.spin()
