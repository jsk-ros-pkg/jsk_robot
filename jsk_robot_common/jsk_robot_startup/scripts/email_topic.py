#!/usr/bin/env python

from email.mime.application import MIMEApplication
from email.mime.image import MIMEImage
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
    attached_files:
      - /home/user/Pictures/test.png
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
        rospy.loginfo('Received an msg: {}'.format(msg))
        send_mail_args = {}
        # Set default value for self._send_mail arguments
        send_mail_args['subject'] = ''
        send_mail_args['body'] = ''
        send_mail_args['sender_address'] = '{}@{}'.format(getpass.getuser(), socket.gethostname())
        send_mail_args['smtp_server'] = 'localhost'
        send_mail_args['smtp_port'] = 25
        send_mail_args['attached_files'] = None
        # Set args from topic field. If the field is empty, use value in yaml
        for field in ['subject', 'body', 'sender_address', 'receiver_address',
                      'smtp_server', 'smtp_port', 'attached_files']:
            if getattr(msg, field) != '':
                send_mail_args[field] = getattr(msg, field)
            else:
                if field in self.email_info:
                    send_mail_args[field] = self.email_info[field]
        # Send email
        try:
            self._send_mail(**send_mail_args)
        except TypeError as e:
            rospy.logerr(e)
            rospy.logerr("'receiver_address' may not be specified.")

    def _send_mail(
            self, subject, body, sender_address, receiver_address,
            smtp_server, smtp_port, attached_files):
        # Create MIME
        msg = MIMEMultipart()
        msg["Subject"] = subject
        msg["From"] = sender_address
        msg["To"] = receiver_address
        # Support embed image
        for content in body:
            if content.type == 'text':
                msg.attach(MIMEText(content.message, 'plain', 'utf-8'))
            elif content.type == 'html':
                msg.attach(MIMEText(content.message, 'html'))
            elif content.type == 'img':
                if content.file_path == '':
                    rospy.logwarn('File name is empty. Skipped.')
                    continue
                if not os.path.exists(content.file_path):
                    rospy.logerr(
                        'File {} is not found.'.format(content.file_path))
                    return
                with open(content.file_path, 'rb') as img:
                    embed_img = MIMEImage(img.read())
                    embed_img.add_header(
                        'Content-ID', '<{}>'.format(content.file_path))
                    msg.attach(embed_img)  # This line is necessary to embed
                if content.img_size:
                    image_size = content.img_size
                else:
                    image_size = 100
                text = '<img src="cid:{}" width={}%>'.format(
                    content.file_path, image_size)
                bodytext = MIMEText(text, 'html')
                msg.attach(bodytext)
            else:
                rospy.logwarn('Unknown content type {}'.format(content.type))
                continue
        # Attach file
        for attached_file in attached_files:
            if attached_file == '':
                rospy.logwarn('File name is empty. Skipped.')
                continue
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
