#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray

def callback(data):
    pub.publish(data)
    
def listener():
    rospy.init_node('marker_msg_from_indigo_to_kinetic', anonymous=True)
    rospy.Subscriber("spots_marker_array", rospy.AnyMsg, callback)
    rospy.spin()
        
if __name__ == '__main__':
    pub = rospy.Publisher('spots_marker_array_kinetic', MarkerArray, queue_size=10)
    listener()
