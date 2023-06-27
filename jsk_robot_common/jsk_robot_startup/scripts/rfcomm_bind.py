#!/usr/bin/env python

import os
import rospy
import subprocess
from subprocess import PIPE
import yaml


class RfcommBind(object):
    """
    This node binds the RFCOMM device to a remote Bluetooth device.
    MAC address of bound devices must be stored in '~rfcomm_devices' file.

    '~rfcomm_devices' yaml file is like the following.
    name is device name for human to distinguish.
    address is device's bluetooth MAC address
    - name: device1
      address: XX:XX:XX:XX:XX:XX
    - name: device2
      address: YY:YY:YY:YY:YY:YY

    Note that this node is called with sudo.

    For detail of rfcomm, run this command:
    $ man rfcomm
    """

    def __init__(self):
        self.rfcomm_devices = {}
        yaml_path = rospy.get_param(
            '~rfcomm_devices', '/var/lib/robot/rfcomm_devices.yaml')
        if os.path.exists(yaml_path):
            with open(yaml_path) as yaml_f:
                self.rfcomm_devices = yaml.load(yaml_f)
            rospy.loginfo('{} is loaded.'.format(yaml_path))
        else:
            rospy.logerr('Cannot find {}'.format(yaml_path))

    def release_devices(self):
        subprocess.call(
            ['sudo', 'rfcomm', 'release', 'all'],
            stdout=PIPE, stderr=PIPE)

    def bind_devices(self):
        dev_num = 0
        for dev in self.rfcomm_devices:
            while 0 == subprocess.call(
                    ['rfcomm', 'show', '{}'.format(
                        dev_num)], stdout=PIPE, stderr=PIPE):
                rospy.loginfo('/dev/rfcomm{} is already used.'.format(dev_num))
                dev_num += 1
            subprocess.call(
                ['sudo', 'rfcomm', 'bind', str(dev_num), dev["address"]],
                stdout=PIPE, stderr=PIPE)
            rospy.loginfo(
                'Device {}({}) is bound to /dev/rfcomm{}'.format(
                    dev["name"], dev["address"], dev_num))
            dev_num += 1


if __name__ == "__main__":
    rospy.init_node('rfcomm_bind')
    app = RfcommBind()
    app.release_devices()
    app.bind_devices()
