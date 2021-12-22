#!/usr/bin/env python

import os
import rospy
from std_msgs.msg import Empty


class Shutdown(object):
    """
    This node shuts down or reboots the robot itself
    according to the rostopic.

    Note that this node needs to be run with sudo privileges.

    Usage:
    # Launch node
    $ su [sudo user] -c ". [setup.bash]; rosrun jsk_robot_startup shutdown.py"

    # To shutdown robot
    rostopic pub /shutdown std_msgs/Empty
    # To restart robot
    rostopic pub /reboot std_msgs/Empty
    """

    def __init__(self):
        rospy.loginfo('Start shutdown node.')
        rospy.Subscriber('shutdown', Empty, self.shutdown)
        rospy.Subscriber('reboot', Empty, self.reboot)

    def shutdown(self, msg):
        rospy.loginfo('Shut down robot.')
        shutdown_command = '/sbin/shutdown -h now'
        ret = os.system(shutdown_command)
        if ret != 0:
            rospy.logerr("Failed to call '$ {}'. Check authentication.".format(
                shutdown_command))

    def reboot(self, msg):
        rospy.loginfo('Reboot robot.')
        reboot_command = '/sbin/shutdown -r now'
        ret = os.system(reboot_command)
        if ret != 0:
            rospy.logerr("Failed to call '$ {}'. Check authentication.".format(
                reboot_command))


if __name__ == '__main__':
    rospy.init_node('shutdown')
    s = Shutdown()
    rospy.spin()
