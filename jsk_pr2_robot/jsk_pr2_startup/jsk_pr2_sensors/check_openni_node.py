#!/usr/bin/env python

import sys,subprocess,traceback
import rospy
import cv
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = None
        
    def callback(self,data):
        self.image_sub.unregister()

        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e

        rospy.loginfo("sum of pixels is %d at %s", max(cv.Sum(cv_image)), data.header.stamp.secs)
        if max(cv.Sum(cv_image)) == 0 :
            rospy.logerr("Restart openni_node1")
            retcode = -1
            try:
                retcode = subprocess.call('rosnode kill /openni/driver', shell=True)
            except Exception, e:
                rospy.logerr('Unable to kill kinect node, caught exception:\n%s', traceback.format_exc())

    def process(self):
        if self.image_sub == None or self.image_sub.impl == None:
            self.image_sub = rospy.Subscriber("/openni/rgb/image_color",Image,self.callback,None,1)


def main(args):
    ic = image_converter()
    rospy.init_node('check_openni_node', anonymous=True)
    while not rospy.is_shutdown():
        ic.process()
        rospy.sleep(3)
          

if __name__ == '__main__':
    main(sys.argv)
