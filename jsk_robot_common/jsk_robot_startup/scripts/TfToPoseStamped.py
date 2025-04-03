#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf

class TfToPoseStamped:
    def __init__(self):
        rospy.init_node("TfToPoseStamped", anonymous=True)
        self.pub = rospy.Publisher("~pose_stamped", PoseStamped, queue_size=10)
        self.parent_frame = rospy.get_param("~parent_frame", "map")
        self.child_frame = rospy.get_param("~child_frame", "base_footprint")
        self.rate = rospy.get_param("~rate", 10)
        self.listener = tf.TransformListener()
        self.r = rospy.Rate(self.rate) # 10hz

    def execute(self):
        while not rospy.is_shutdown():
            self.publish_pose_stamped()
            self.r.sleep()

    def publish_pose_stamped(self):
        try:
            (trans,rot) = self.listener.lookupTransform(self.parent_frame, self.child_frame, rospy.Time(0))
            pub_msg = PoseStamped()
            pub_msg.pose.position.x = trans[0]
            pub_msg.pose.position.y = trans[1]
            pub_msg.pose.position.z = trans[2]
            pub_msg.pose.orientation.x = rot[0]
            pub_msg.pose.orientation.y = rot[1]
            pub_msg.pose.orientation.z = rot[2]
            pub_msg.pose.orientation.w = rot[3]
            pub_msg.header.stamp = rospy.Time.now()
            pub_msg.header.frame_id = self.parent_frame
            self.pub.publish(pub_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

if __name__ == '__main__':
    try:
        node = TfToPoseStamped()
        node.execute()
    except rospy.ROSInterruptException: pass
