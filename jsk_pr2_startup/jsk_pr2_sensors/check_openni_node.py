#!/usr/bin/env python

import roslib
roslib.load_manifest('jsk_pr2_startup')
import sys,subprocess,traceback
import rospy
import cv
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        
    def callback(self,data):
        self.image_sub.unregister()

        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e

        rospy.loginfo("sum of pixels %d at %s.%s", max(cv.Sum(cv_image)), data.header.stamp.secs, data.header.stamp.nsecs)
        if max(cv.Sum(cv_image)) == 0 :
            rospy.logerr("Restart openni_node1")
            retcode = -1
            try:
                retcode = subprocess.call('rosnode kill openni_node1', shell=True)
            except Exception, e:
                rospy.logerr('Unable to kill kinect node, caught exception:\n%s', traceback.format_exc())

    def process(self):
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)


def main(args):
    ic = image_converter()
    rospy.init_node('check_openni_node', anonymous=True)
    while not rospy.is_shutdown():
        ic.process()
        rospy.sleep(1)
          

if __name__ == '__main__':
    main(sys.argv)
