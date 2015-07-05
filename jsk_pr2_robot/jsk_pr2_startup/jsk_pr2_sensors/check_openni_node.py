#!/usr/bin/env python

import sys,subprocess,traceback
import rospy
import cv
import time
import os
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from sound_play.msg import SoundRequest

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = None
        self.camera = rospy.get_param('~camera', 'kinect_head')

    def callback(self,data):
        if data.header.stamp < self.prev_time + rospy.Duration(60):
            return
        self.image_sub.unregister()
        self.prev_time = data.header.stamp
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e

        rospy.loginfo("sum of pixels is %d at %s", max(cv.Sum(cv_image)), data.header.stamp.secs)
        if max(cv.Sum(cv_image)) == 0 :
        #if True:
            rospy.logerr("Restart openni_node1")
            retcode = -1
            try:
                #retcode = subprocess.call('rosnode kill /%s/driver' % self.camera, shell=True)
                # 3. usbreset...
                speak("resetting u s b")
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
                speak("something wrong with kinect, I'll restart it, killing nodelet manager")
                retcode = subprocess.call('rosnode kill /%s/%s_nodelet_manager' % (self.camera, self.camera), shell=True)
                time.sleep(10)
                # 2. pkill
                speak("killing child processes")
                retcode = subprocess.call('pkill -f %s_nodelet_manager' % self.camera, shell=True)
                time.sleep(10)
                # 3 restarting
                speak("restarting processes")
                retcode = subprocess.call('roslaunch openni_launch openni.launch camera:=%s publish_tf:=false depth_registration:=true rgb_processing:=false ir_processing:=false depth_processing:=false depth_registered_processing:=false disparity_processing:=false disparity_registered_processing:=false hw_registered_processing:=true sw_registered_processing:=false rgb_frame_id:=/head_mount_kinect_rgb_optical_frame depth_frame_id:=/head_mount_kinect_ir_optical_frame' % self.camera, shell=True)
            except Exception, e:
                rospy.logerr('Unable to kill kinect node, caught exception:\n%s', traceback.format_exc())

    def process(self):
        if self.image_sub == None or self.image_sub.impl == None:
            self.image_sub = rospy.Subscriber("image",Image,self.callback,None,1)

speak_pub = None
def speak(string):
    global speak_pub
    rospy.logerr(string)
    msg = SoundRequest()
    msg.sound = SoundRequest.SAY
    msg.command = SoundRequest.PLAY_ONCE
    msg.arg = string
    speak_pub.publish(msg)


def main(args):
    global speak_pub
    ic = image_converter()
    rospy.init_node('check_openni_node', anonymous=True)
    ic.prev_time = rospy.Time.now()
    speak_pub = rospy.Publisher("/robotsound", SoundRequest)
    while not rospy.is_shutdown():
        ic.process()
        rospy.sleep(3)
          

if __name__ == '__main__':
    main(sys.argv)
