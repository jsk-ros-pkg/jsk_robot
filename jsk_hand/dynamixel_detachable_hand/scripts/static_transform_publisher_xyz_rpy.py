#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _float_triplet(parser: argparse.ArgumentParser, values: list[str], name: str) -> tuple[float, float, float]:
    if len(values) == 1:
        values = values[0].split()
    if len(values) != 3:
        parser.error(f"argument {name}: expected 3 values")
    return tuple(float(value) for value in values)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xyz", nargs="+", default=("0", "0", "0"))
    parser.add_argument("--rpy", nargs="+", default=("0", "0", "0"))
    parser.add_argument("--frame-id", required=True)
    parser.add_argument("--child-frame-id", required=True)
    args, _ = parser.parse_known_args()
    args.xyz = _float_triplet(parser, args.xyz, "--xyz")
    args.rpy = _float_triplet(parser, args.rpy, "--rpy")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = Node("static_transform_publisher_xyz_rpy")
    broadcaster = StaticTransformBroadcaster(node)

    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = args.frame_id
    transform.child_frame_id = args.child_frame_id
    transform.transform.translation.x = args.xyz[0]
    transform.transform.translation.y = args.xyz[1]
    transform.transform.translation.z = args.xyz[2]
    qx, qy, qz, qw = quaternion_from_rpy(*args.rpy)
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw

    broadcaster.sendTransform(transform)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
