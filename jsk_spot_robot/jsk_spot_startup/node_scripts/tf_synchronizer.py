#!/usr/bin/env python3

import message_filters
import rospy
import tf.transformations
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


def invert_transform(orig: TransformStamped) -> TransformStamped:
    inv = TransformStamped()
    inv.header.stamp = orig.header.stamp
    inv.header.frame_id = orig.child_frame_id
    inv.child_frame_id = orig.header.frame_id

    # inverse quaternion
    q_inv = tf.transformations.quaternion_inverse(
        (
            orig.transform.rotation.x,
            orig.transform.rotation.y,
            orig.transform.rotation.z,
            orig.transform.rotation.w,
        )
    )
    (
        inv.transform.rotation.x,
        inv.transform.rotation.y,
        inv.transform.rotation.z,
        inv.transform.rotation.w,
    ) = q_inv

    # −R_inv * t
    tv = orig.transform.translation
    mat = tf.transformations.quaternion_matrix(q_inv)
    x, y, z, _ = mat.dot([tv.x, tv.y, tv.z, 1.0])
    inv.transform.translation.x = -x
    inv.transform.translation.y = -y
    inv.transform.translation.z = -z

    return inv


def callback(robot_tf: TFMessage, driver_tf: TFMessage) -> None:
    # frames only exist in robot_state_publisher
    chain = [
        "arm0.link_wr1",
        "arm0.link_wr0",
        "arm0.link_el1",
        "arm0.link_el0",
    ]
    skip_set = set(chain)

    orig_map = {
        t.child_frame_id: t for t in robot_tf.transforms if t.child_frame_id in skip_set
    }

    root = next((t for t in driver_tf.transforms if t.child_frame_id == chain[0]), None)

    combined = TFMessage()

    combined.transforms.append(root)

    for child in chain:
        orig = orig_map.get(child)
        if orig:
            combined.transforms.append(invert_transform(orig))

    for t in driver_tf.transforms:
        if t.child_frame_id not in skip_set and t.child_frame_id != chain[0]:
            combined.transforms.append(t)

    for t in robot_tf.transforms:
        if t.child_frame_id not in skip_set:
            combined.transforms.append(t)

    tf_pub.publish(combined)


if __name__ == "__main__":
    rospy.init_node("tf_synchronizer")
    tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=10)

    sub1 = message_filters.Subscriber("/robot_state_publisher/tf", TFMessage)
    sub2 = message_filters.Subscriber("/spot/spot_ros/tf", TFMessage)
    ats = message_filters.ApproximateTimeSynchronizer(
        [sub1, sub2], queue_size=10, slop=0.1, allow_headerless=True
    )
    ats.registerCallback(callback)
    rospy.spin()
