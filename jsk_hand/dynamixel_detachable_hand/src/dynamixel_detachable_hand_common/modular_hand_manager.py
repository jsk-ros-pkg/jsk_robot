from __future__ import annotations

import argparse
import sys
from typing import Iterable

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

from dynamixel_detachable_hand.srv import GetToolState, SetTool
from dynamixel_detachable_hand_common.description import (
    HandModel,
    describe_hand,
    load_tool_config,
    normalize_side,
    normalize_tool,
    resolve_tool_from_ids,
    scan_ids_from_config,
)
from dynamixel_detachable_hand_common.messages import description_to_msg, state_to_msg


TRANSIENT_LOCAL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ModularHandManager(Node):
    def __init__(
        self,
        *,
        side: str,
        tool: str = "generic",
        attached: bool = True,
        include_detacher: bool = True,
        port_name: str | None = None,
        baud_rate: int | str | None = None,
        protocol_version: str | float | None = None,
        tool_config_file: str | None = None,
        robot_state_publisher_node: str = "robot_state_publisher",
        auto_detect: bool = False,
        scan_ids: Iterable[int] | None = None,
        detect_period: float = 1.0,
        simulated_present_ids: Iterable[int] | None = None,
    ) -> None:
        side = normalize_side(side)
        tool = normalize_tool(tool, attached=attached)
        super().__init__(f"{side}_modular_hand_manager")
        self.side = side
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.protocol_version = protocol_version
        self.tool_config_file = tool_config_file
        self.tool_config = load_tool_config(tool_config_file)
        self.include_detacher = include_detacher
        self.robot_state_publisher_node = robot_state_publisher_node
        self.scan_ids = tuple(scan_ids or scan_ids_from_config(side, tool_config=self.tool_config))
        self.simulated_present_ids = tuple(simulated_present_ids) if simulated_present_ids is not None else None
        self.model = self._describe(tool=tool, attached=attached)
        self._parameter_futures: list[Future] = []

        self.robot_description_pub = self.create_publisher(String, "robot_description", TRANSIENT_LOCAL_QOS)
        self.tool_state_pub = self.create_publisher(type(state_to_msg(self.model, self.get_clock().now().to_msg())), "tool_state", TRANSIENT_LOCAL_QOS)
        self.hand_description_pub = self.create_publisher(type(description_to_msg(self.model, self.get_clock().now().to_msg())), "hand_description", TRANSIENT_LOCAL_QOS)

        self.create_service(SetTool, "set_tool", self._set_tool_cb)
        self.create_service(GetToolState, "get_tool_state", self._get_tool_state_cb)
        self.publish_model()
        if auto_detect:
            self.get_logger().info(f"{self.side}: auto detect enabled, scan_ids={list(self.scan_ids)}")
            self.create_timer(max(detect_period, 0.1), self.detect_and_update)

    def _describe(self, *, tool: str, attached: bool) -> HandModel:
        return describe_hand(
            self.side,
            tool,
            attached=attached,
            include_detacher=self.include_detacher,
            port_name=self.port_name,
            baud_rate=self.baud_rate,
            protocol_version=self.protocol_version,
            tool_config=self.tool_config,
        )

    def publish_model(self) -> None:
        stamp = self.get_clock().now().to_msg()
        description = String()
        description.data = self.model.robot_description
        self.robot_description_pub.publish(description)
        self.tool_state_pub.publish(state_to_msg(self.model, stamp))
        self.hand_description_pub.publish(description_to_msg(self.model, stamp))
        self.get_logger().info(
            f"{self.side}: tool={self.model.tool}, attached={self.model.attached}, "
            f"tip={self.model.tip_link}, motors={[m.id for m in self.model.motors]}"
        )

    def set_model(self, *, tool: str, attached: bool, update_robot_state_publisher: bool = False) -> HandModel:
        model = self._describe(tool=tool, attached=attached)
        if model.robot_description_sha256 == self.model.robot_description_sha256:
            return self.model
        self.model = model
        self.publish_model()
        if update_robot_state_publisher:
            ok, message = self._request_robot_state_publisher_update(model)
            if not ok:
                self.get_logger().warn(message)
        return model

    def detect_and_update(self) -> None:
        present_ids = self.simulated_present_ids
        if present_ids is None:
            present_ids = ping_present_ids(self.port_name or f"/dev/{self.side}", self.baud_rate, self.scan_ids)
        tool, attached = resolve_tool_from_ids(self.side, present_ids, tool_config=self.tool_config)
        if tool != self.model.tool or attached != self.model.attached:
            self.get_logger().info(f"{self.side}: detected ids={list(present_ids)} -> tool={tool}, attached={attached}")
            self.set_model(tool=tool, attached=attached, update_robot_state_publisher=True)

    def _set_tool_cb(self, request: SetTool.Request, response: SetTool.Response) -> SetTool.Response:
        if request.side and normalize_side(request.side) != self.side:
            response.success = False
            response.message = f"manager for {self.side} cannot update {request.side}"
            return response
        try:
            model = self.set_model(
                tool=request.tool,
                attached=request.attached,
                update_robot_state_publisher=request.update_robot_state_publisher,
            )
            response.success = True
            response.message = f"{self.side} set to {model.tool}, attached={model.attached}"
            response.state = state_to_msg(model, self.get_clock().now().to_msg())
            response.robot_description = model.robot_description
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _get_tool_state_cb(self, request: GetToolState.Request, response: GetToolState.Response) -> GetToolState.Response:
        if request.side and normalize_side(request.side) != self.side:
            response.success = False
            response.message = f"manager for {self.side} cannot report {request.side}"
            return response
        stamp = self.get_clock().now().to_msg()
        response.success = True
        response.message = "ok"
        response.state = state_to_msg(self.model, stamp)
        response.hand_description = description_to_msg(self.model, stamp)
        response.robot_description = self.model.robot_description
        return response

    def _request_robot_state_publisher_update(self, model: HandModel) -> tuple[bool, str]:
        service_name = f"{self.robot_state_publisher_node}/set_parameters"
        client = self.create_client(SetParameters, service_name)
        if not client.wait_for_service(timeout_sec=1.0):
            return False, f"{service_name} is not available"
        request = SetParameters.Request()
        value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=model.robot_description)
        request.parameters = [Parameter(name="robot_description", value=value)]
        future = client.call_async(request)
        self._parameter_futures.append(future)
        return True, f"requested robot_state_publisher update through {service_name}"


def ping_present_ids(port_name: str, baud_rate: int | str | None, ids: Iterable[int]) -> tuple[int, ...]:
    try:
        from dynamixel_sdk import PacketHandler, PortHandler
    except Exception:
        return ()

    port = PortHandler(port_name)
    if not port.openPort():
        return ()
    try:
        port.setBaudRate(int(baud_rate or 57600))
        packet = PacketHandler(2.0)
        present: list[int] = []
        for motor_id in ids:
            _model_number, result, _error = packet.ping(port, int(motor_id))
            if result == 0:
                present.append(int(motor_id))
        return tuple(present)
    finally:
        port.closePort()


def _parse_csv_ints(text: str | None) -> tuple[int, ...] | None:
    if text is None or text == "":
        return None
    return tuple(int(item.strip(), 0) for item in text.split(",") if item.strip())


def _parse_bool(text: str | bool) -> bool:
    if isinstance(text, bool):
        return text
    return text.strip().lower() in ("1", "true", "yes", "on")


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=list(("lhand", "rhand")), required=True)
    parser.add_argument("--tool", default="generic")
    parser.add_argument("--attached", default="true")
    parser.add_argument("--include-detacher", default="true")
    parser.add_argument("--port-name")
    parser.add_argument("--baud-rate")
    parser.add_argument("--protocol-version")
    parser.add_argument("--tool-config-file")
    parser.add_argument("--robot-state-publisher-node", default="robot_state_publisher")
    parser.add_argument("--auto-detect", default="false")
    parser.add_argument("--scan-ids")
    parser.add_argument("--detect-period", type=float, default=1.0)
    parser.add_argument("--simulated-present-ids")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    args = _parse_args(remove_ros_args(sys.argv if argv is None else argv)[1:])
    node = ModularHandManager(
        side=args.side,
        tool=args.tool,
        attached=_parse_bool(args.attached),
        include_detacher=_parse_bool(args.include_detacher),
        port_name=args.port_name,
        baud_rate=args.baud_rate,
        protocol_version=args.protocol_version,
        tool_config_file=args.tool_config_file,
        robot_state_publisher_node=args.robot_state_publisher_node,
        auto_detect=_parse_bool(args.auto_detect),
        scan_ids=_parse_csv_ints(args.scan_ids),
        detect_period=args.detect_period,
        simulated_present_ids=_parse_csv_ints(args.simulated_present_ids),
    )
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
