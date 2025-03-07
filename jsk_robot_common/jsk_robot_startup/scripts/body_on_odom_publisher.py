#!/usr/bin/env python

import rospy
import tf
import time

def update_position():
    global position
    global orientation
    global stamp

    rospy.logdebug("update body_on_odom")
    try:
        # stamp = rospy.Time(0)
        tfl.waitForTransform(robot_frame, odom_ground_frame, stamp, rospy.Duration(1.0))
        (trans, rot) = tfl.lookupTransform(odom_ground_frame, robot_frame,  stamp)
        position = [0, 0, -trans[2]]
        rot_euler = tf.transformations.euler_from_quaternion(rot)
        q = tf.transformations.quaternion_from_euler(rot_euler[0], rot_euler[1], 0)
        orientation = [-q[0], -q[1], -q[2], q[3]] # conjugated
    except:
        rospy.logerr("Failed to update body_on_odom")
        return

if __name__ == "__main__":
    rospy.init_node("body_on_odom_publisher")
    # tfl = tf.TransformListener(True, rospy.Duration(10))
    tfl = tf.TransformListener()
    tfb = tf.TransformBroadcaster()
    robot_frame = rospy.get_param("~robot_frame", "BODY")
    rospy.loginfo("robot frame: {}".format(robot_frame))
    odom_ground_frame = rospy.get_param("~odom_ground_frame", "odom_ground")
    rospy.loginfo("odom ground frame: {}".format(odom_ground_frame))
    rate = rospy.get_param("~rate", 100)
    r = rospy.Rate(rate)

    position = None
    orientation = None

    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        update_position()
        try:
            tfb.sendTransform(position, orientation, stamp, "body_on_odom", robot_frame)
            # tfb.sendTransform(position, orientation, rospy.Time(0), "body_on_odom", robot_frame)
        except:
            rospy.logerr("Failed to broadcast {} to {} transform".format("body_on_odom", robot_frame))
            time.sleep(1)
        r.sleep()
