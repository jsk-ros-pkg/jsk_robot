import rospy
from jsk_robot_startup.msg import Email
from jsk_robot_startup.msg import EmailBody

class EmailTopicClient:

    def __init__(self, email_topic='email'):

        self.pub = rospy.Publisher(email_topic, Email, queue_size=1)

    def send_mail(self,
                  subject,
                  receiver_address,
                  body,
                  sender_address=None,
                  smtp_server=None,
                  smtp_port=None,
                  attached_files=None):

        msg = Email()
        msg.header.stamp = rospy.Time.now()
        msg.subject = subject
        email_body = EmailBody()
        email_body.type = "text"
        email_body.message = body
        msg.body.append(email_body)
        msg.receiver_address = receiver_address
        if sender_address is not None:
            msg.sender_address = sender_address
        if smtp_server is not None:
            msg.smtp_server = smtp_server
        if smtp_port is not None:
            msg.smtp_port = smtp_port
        if attached_files is not None:
            msg.attached_files = attached_files
        self.pub.publish(msg)
