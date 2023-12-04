#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf

class PoseStampedToTf:
    def __init__(self):
        rospy.init_node("PoseStampedToTf", anonymous=True)
        self.frame_name = rospy.get_param("~frame_name", "/pose_stamped")
        self.rate = rospy.get_param("~rate", 10)
        self.broadcast = tf.TransformBroadcaster()        
        self.r = rospy.Rate(self.rate) # 10hz
        self.sub = rospy.Subscriber("~input_pose", PoseStamped, self.callback)

    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def callback(self, msg):
        self.broadcast.sendTransform([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                                     [msg.pose.orientation.x, msg.pose.orientation.y, 
                                      msg.pose.orientation.z, msg.pose.orientation.w],
                                     msg.header.stamp, self.frame_name, msg.header.frame_id)
            
if __name__ == '__main__':
    try:
        node = PoseStampedToTf()
        node.execute()
    except rospy.ROSInterruptException: pass
