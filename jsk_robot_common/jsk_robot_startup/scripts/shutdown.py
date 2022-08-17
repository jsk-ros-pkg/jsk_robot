#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from sound_play.libsoundplay import SoundClient
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
        self.client_jp = SoundClient(sound_action='/robotsound_jp', blocking=True)
        rospy.Subscriber('shutdown', Empty, self.shutdown)
        rospy.Subscriber('reboot', Empty, self.reboot)
        self.shutdown_command = rospy.get_param(
            '~shutdown_command', '/sbin/shutdown -h now')
        self.reboot_command = rospy.get_param(
            '~reboot_command', '/sbin/shutdown -r now')
        self.volume = rospy.get_param('~volume', 1.0)

    def speak(self, client, speech_text, lang='jp'):
        client.say(speech_text, voice=lang, volume=self.volume, replace=False)
        return client.actionclient.get_result()

    def shutdown(self, msg):
        rospy.loginfo('Shut down robot.')
        self.speak(self.client_jp, 'シャットダウンします。')
        ret = os.system(self.shutdown_command)
        if ret != 0:
            rospy.logerr("Failed to call '$ {}'. Check authentication.".format(
                self.shutdown_command))

    def reboot(self, msg):
        rospy.loginfo('Reboot robot.')
        self.speak(self.client_jp, '再起動します。')
        ret = os.system(self.reboot_command)
        if ret != 0:
            rospy.logerr("Failed to call '$ {}'. Check authentication.".format(
                self.reboot_command))


if __name__ == '__main__':
    rospy.init_node('shutdown')
    s = Shutdown()
    rospy.spin()
