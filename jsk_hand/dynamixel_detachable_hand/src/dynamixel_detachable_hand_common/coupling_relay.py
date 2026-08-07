from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from dynamixel_detachable_hand_common.description import (
    ToolCoupling,
    evaluate_polynomial,
    extract_couplings,
)


TRANSIENT_LOCAL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def apply_couplings(msg: JointState, couplings: tuple[ToolCoupling, ...]) -> JointState:
    out = JointState()
    out.header = msg.header
    out.name = list(msg.name)
    out.position = _resize_float_array(msg.position, len(out.name))
    out.velocity = _resize_float_array(msg.velocity, len(out.name))
    out.effort = _resize_float_array(msg.effort, len(out.name))
    positions = {name: out.position[index] for index, name in enumerate(out.name)}
    index_by_name = {name: index for index, name in enumerate(out.name)}

    for coupling in couplings:
        if coupling.driver not in positions:
            continue
        value = evaluate_polynomial(coupling.poly, positions[coupling.driver])
        if coupling.joint in index_by_name:
            out.position[index_by_name[coupling.joint]] = value
        else:
            out.name.append(coupling.joint)
            out.position.append(value)
            out.velocity.append(0.0)
            out.effort.append(0.0)
    return out


class JointStateCouplingRelay(Node):
    def __init__(self, source_topic: str, output_topic: str) -> None:
        super().__init__("joint_state_coupling_relay")
        self.couplings: tuple[ToolCoupling, ...] = ()
        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.create_subscription(String, "robot_description", self._robot_description_cb, TRANSIENT_LOCAL_QOS)
        self.create_subscription(JointState, source_topic, self._joint_state_cb, 10)

    def _robot_description_cb(self, msg: String) -> None:
        try:
            self.couplings = extract_couplings(msg.data)
            self.get_logger().info(f"loaded {len(self.couplings)} joint coupling rules")
        except Exception as exc:
            self.couplings = ()
            self.get_logger().error(f"failed to parse coupling metadata: {exc}")

    def _joint_state_cb(self, msg: JointState) -> None:
        self.publisher.publish(apply_couplings(msg, self.couplings))


def _resize_float_array(values, size: int) -> list[float]:
    output = [float(value) for value in values]
    if not output:
        return [0.0] * size
    if len(output) < size:
        output.extend([0.0] * (size - len(output)))
    return output[:size]


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--joint-states-source-topic", default="joint_states_source")
    parser.add_argument("--joint-states-topic", default="joint_states")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    args = _parse_args(remove_ros_args(sys.argv if argv is None else argv)[1:])
    node = JointStateCouplingRelay(args.joint_states_source_topic, args.joint_states_topic)
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
