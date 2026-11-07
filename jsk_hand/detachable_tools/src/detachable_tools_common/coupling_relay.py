from __future__ import annotations

import argparse

from sensor_msgs.msg import JointState
from std_msgs.msg import String

from detachable_tools_common.description import (
    ToolCoupling,
    evaluate_polynomial,
    extract_couplings,
)
from detachable_tools_common.ros_compat import node_argv, ros_compat


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


class JointStateCouplingRelay:
    """Fill the display-only joints from the <coupling> metadata of robot_description."""

    def __init__(self, node, source_topic: str, output_topic: str) -> None:
        self.node = node
        self.couplings: tuple[ToolCoupling, ...] = ()
        self.publisher = ros_compat.create_publisher(node, output_topic, JointState, queue_size=10)
        self._description_sub = ros_compat.create_subscription(
            node, "robot_description", String, self._robot_description_cb, queue_size=1, latch=True
        )
        self._joint_state_sub = ros_compat.create_subscription(
            node, source_topic, JointState, self._joint_state_cb, queue_size=10
        )

    def _robot_description_cb(self, msg: String) -> None:
        try:
            self.couplings = extract_couplings(msg.data)
            ros_compat.loginfo(self.node, f"loaded {len(self.couplings)} joint coupling rules")
        except Exception as exc:
            self.couplings = ()
            ros_compat.logerr(self.node, f"failed to parse coupling metadata: {exc}")

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
    args = _parse_args(node_argv(argv))
    node = ros_compat.init_node("joint_state_coupling_relay")
    JointStateCouplingRelay(node, args.joint_states_source_topic, args.joint_states_topic)
    ros_compat.spin(node)


if __name__ == "__main__":
    main()
