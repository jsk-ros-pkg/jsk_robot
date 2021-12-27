#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from jsk_robot_startup.msg import Email


class PowerManagement(object):
    """
    This node manages the power system of the robot.

    Usage:
    # Launch shutdown node
    $ su [sudo user] -c ". [setup.bash]; rosrun jsk_robot_startup shutdown.py"

    # Launch power management node and email node
    $ roslaunch jsk_robot_startup power_management.launch

    # Shutdown robot after notifying with email
    $ rostopic pub /power_management/shutdown jsk_robot_startup/Email "header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    subject: ''
    body: ''
    sender_address: ''
    receiver_address: ''
    smtp_server: ''
    smtp_port: ''
    attached_files: []"
    """

    def __init__(self):
        rospy.loginfo('Start power management node.')
        rospy.Subscriber(
            '~shutdown', Email, self.shutdown)
        rospy.Subscriber(
            '~reboot', Email, self.reboot)
        self.shutdown_pub = rospy.Publisher('shutdown', Empty, queue_size=1)
        self.reboot_pub = rospy.Publisher('reboot', Empty, queue_size=1)
        self.email_pub = rospy.Publisher('email', Email, queue_size=1)

    def shutdown(self, msg):
        rospy.loginfo('Shut down robot after notifying with email.')
        self.email_pub.publish(msg)
        self.shutdown_pub.publish(Empty())

    def reboot(self, msg):
        rospy.loginfo('Reboot robot after notifying with email.')
        self.email_pub.publish(msg)
        self.reboot_pub.publish(Empty())


if __name__ == '__main__':
    rospy.init_node('power_management')
    s = PowerManagement()
    rospy.spin()
