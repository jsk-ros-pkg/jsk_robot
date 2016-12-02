#!/usr/bin/env python

import sys,subprocess,traceback
import rospy
import cv
import time
import os
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from sound_play.msg import SoundRequest

class CheckOpenNINode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = None
        self.camera = rospy.get_param('~camera', 'kinect_head')
        self.sleep_cycle = rospy.get_param('~sleep_cycle', 60)
        self.speak_enabled = rospy.get_param("~speak", True)
        self.speak_pub = rospy.Publisher("/robotsound", SoundRequest)
        self.restart_srv = rospy.Service('~restart', Empty, self.restart_service_callback)

    def speak(self, speak_str):
        rospy.logerr("[%s] %s", self.__class__.__name__, speak_str)
        if self.speak_enabled:
            msg = SoundRequest()
            msg.sound = SoundRequest.SAY
            msg.command = SoundRequest.PLAY_ONCE
            msg.arg = speak_str
            self.speak_pub.publish(msg)

    def restart_service_callback(self, req):
        self.restart_openni_node()
        return EmptyResponse()

    def image_callback(self, msg):
        self.image_sub.unregister()
        try:
            cv_image = self.bridge.imgmsg_to_cv(msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("[%s] failed to convert image to cv", self.__class__.__name__)
            return

        sum_of_pixels = max(cv.Sum(cv_image))
        rospy.loginfo("[%s] sum of pixels is %d at %s", self.__class__.__name__, sum_of_pixels, msg.header.stamp.secs)
        if sum_of_pixels == 0:
            self.restart_openni_node()

    def restart_openni_node(self):
        rospy.logerr("Restart openni_node1")
        retcode = -1
        try:
            # 3. usbreset...
            self.speak("resetting u s b")
            p = subprocess.Popen("lsusb", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = p.communicate()
            lines = stdout.split("\n")
            ms_line = [l for l in lines if "Microsoft" in l][0]
            # it should be like Bus 002 Device 013: ID 045e:02ad Microsoft Corp. Xbox NUI Audio
            bus_id = ms_line.split(' ')[1]
            bus_device_dir = "/dev/bus/usb/" + bus_id
            files = os.listdir(bus_device_dir)
            for f in files:
                full_path = os.path.join(bus_device_dir, f)
                if os.access(full_path, os.W_OK):
                    retcode = subprocess.call('rosrun openni2_camera usb_reset ' + full_path, shell=True)
            time.sleep(10)
            # 1. kill nodelet manager
            self.speak("something wrong with kinect, I'll restart it, killing nodelet manager")
            retcode = subprocess.call('rosnode kill /%s/%s_nodelet_manager' % (self.camera, self.camera), shell=True)
            time.sleep(10)
            # 2. pkill
            self.speak("killing child processes")
            retcode = subprocess.call('pkill -f %s_nodelet_manager' % self.camera, shell=True)
            time.sleep(10)
            # 3 restarting
            self.speak("restarting processes")
            retcode = subprocess.call('roslaunch openni_launch openni.launch camera:=%s publish_tf:=false depth_registration:=true rgb_processing:=false ir_processing:=false depth_processing:=false depth_registered_processing:=false disparity_processing:=false disparity_registered_processing:=false hw_registered_processing:=true sw_registered_processing:=false rgb_frame_id:=/head_mount_kinect_rgb_optical_frame depth_frame_id:=/head_mount_kinect_ir_optical_frame' % self.camera, shell=True)
        except Exception, e:
            rospy.logerr('[%s] Unable to kill kinect node, caught exception:\n%s', self.__class__.__name__, traceback.format_exc())

    def run(self):
        while not rospy.is_shutdown():
            if self.image_sub == None or self.image_sub.impl == None:
                self.image_sub = rospy.Subscriber("image", Image, self.image_callback, None, 1)
            rospy.sleep(self.sleep_cycle)


if __name__ == '__main__':
    rospy.init_node('check_openni_node', anonymous=True)
    ic = CheckOpenNINode()
    ic.run()
