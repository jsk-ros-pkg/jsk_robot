"""Switch the detachable tool with set_tool/<tool> without restarting the launch (ROS 1 / ROS 2)."""

from __future__ import annotations

import argparse

from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from std_srvs.srv import Trigger

from detachable_tools_common.description import (
    TOOLS,
    VALID_TOOLS,
    HandModel,
    build_control_description,
    component_name,
    describe_hand,
    gripper_controller_name,
    normalize_side,
    normalize_tool,
)
from detachable_tools_common.messages import description_to_msg, detection_to_msg, state_to_msg, state_to_text
from detachable_tools_common.ros_compat import ROS_VERSION, node_argv, ros_compat

JOINT_STATE_BROADCASTER = "joint_state_broadcaster"
DETACHER_CONTROLLER = "joint_group_effort_controller"


class ModularHandManager:
    """``node`` is the rclpy node on ROS 2 and ``None`` on ROS 1."""

    def __init__(
        self,
        *,
        side: str,
        node=None,
        tool: str = "generic",
        attached: bool = True,
        include_detacher: bool = True,
        port_name: str | None = None,
        baud_rate: int | str | None = None,
        protocol_version: str | float | None = None,
        robot_state_publisher_node: str = "robot_state_publisher",
        controller_manager: str = "controller_manager",
        use_ros2_control: bool | None = None,
        restart_hardware: bool = False,
        hand_effort_limit: float = 0.92,
        detected_ids: list[int] | None = None,
    ) -> None:
        self.side = normalize_side(side)
        self.node = node
        self.include_detacher = include_detacher
        self.hardware = dict(port_name=port_name, baud_rate=baud_rate, protocol_version=protocol_version)
        self.robot_state_publisher_node = robot_state_publisher_node
        self.controller_manager = controller_manager
        self.use_ros2_control = (ROS_VERSION == 2) if use_ros2_control is None else use_ros2_control
        self._group = None
        self.restart_hardware = ROS_VERSION == 1 and restart_hardware
        self.hand_effort_limit = hand_effort_limit
        self.model = self._describe(tool=normalize_tool(tool, attached=attached), attached=attached)

        self.tool_state_pub = ros_compat.create_publisher(node, "tool_state", DiagnosticArray, latch=True)
        self.hand_description_pub = ros_compat.create_publisher(node, "hand_description", DiagnosticArray, latch=True)
        if ROS_VERSION == 1:
            # ROS 2 robot_state_publisher publishes robot_description itself
            self.robot_description_pub = ros_compat.create_publisher(node, "robot_description", String, latch=True)
        if self.use_ros2_control:
            self._init_ros2_control()
        if self.restart_hardware:
            self._ros1_timer = ros_compat.create_timer(node, 0.1, self._start_ros1_controllers)
        self._services = [self._service("get_tool_state", self._get_tool_state_cb)]
        for tool_name in VALID_TOOLS:
            self._services.append(self._service(f"set_tool/{tool_name}", self._make_set_tool_cb(tool_name)))
        self.publish_model()
        if detected_ids is not None:
            self.detected_tool_pub = ros_compat.create_publisher(node, "detected_tool", DiagnosticArray, latch=True)
            self.detected_tool_pub.publish(detection_to_msg(
                self.model, ros_compat.stamp_now(node), detected_ids, port_name or f"/dev/{self.side}"
            ))

    # -- ROS 2 control -------------------------------------------------------
    def _init_ros2_control(self) -> None:
        from controller_manager_msgs.srv import (
            ConfigureController,
            ListControllers,
            ListHardwareComponents,
            LoadController,
            SetHardwareComponentState,
            SwitchController,
        )
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._group = ReentrantCallbackGroup()
        cm = self.controller_manager
        self._set_hw = self.node.create_client(
            SetHardwareComponentState, f"{cm}/set_hardware_component_state", callback_group=self._group
        )
        self._switch = self.node.create_client(SwitchController, f"{cm}/switch_controller", callback_group=self._group)
        self._list = self.node.create_client(ListControllers, f"{cm}/list_controllers", callback_group=self._group)
        self._list_hw = self.node.create_client(
            ListHardwareComponents, f"{cm}/list_hardware_components", callback_group=self._group
        )
        self._load = self.node.create_client(LoadController, f"{cm}/load_controller", callback_group=self._group)
        self._configure = self.node.create_client(
            ConfigureController, f"{cm}/configure_controller", callback_group=self._group
        )
        description = String()
        description.data = build_control_description(self.side, include_detacher=self.include_detacher, **self.hardware)
        # controller_manager subscribes to it (remapped from robot_description)
        self.control_description_pub = ros_compat.create_publisher(
            self.node, "control_description", String, latch=True
        )
        self.control_description_pub.publish(description)
        self._started = False
        self._start_timer = self.node.create_timer(0.5, self._start_ros2_control, callback_group=self._group)

    def _start_ros2_control(self) -> None:
        """Load the controllers of every tool, then activate the detacher and the attached tool."""
        from controller_manager_msgs.srv import ConfigureController, ListHardwareComponents, LoadController

        if self._started:
            return
        # Claim startup before blocking: this timer uses a reentrant callback group.
        self._started = True
        response = self._call(self._list_hw, ListHardwareComponents.Request(), timeout=1.0)
        if not response or not response.component:
            self._started = False
            return  # control_description not received yet
        self._start_timer.cancel()
        loaded = self._loaded_controllers()
        controllers = [JOINT_STATE_BROADCASTER] + [gripper_controller_name(t) for t in TOOLS]
        if self.include_detacher:
            controllers.append(DETACHER_CONTROLLER)
        for name in controllers:
            if name in loaded:
                continue
            ok = self._call(self._load, LoadController.Request(name=name))
            ok = ok and ok.ok and self._call(self._configure, ConfigureController.Request(name=name))
            if not (ok and ok.ok):
                ros_compat.logwarn(self.node, f"{name} could not be loaded (config/{self.side}/controllers.yaml?)")
        if self.include_detacher:
            self._set_component(component_name(self.side, "detacher"), "active")
            self._switch_controllers(activate=[DETACHER_CONTROLLER])
        ok, message = self._activate(self.model)
        if not ok:
            ros_compat.logwarn(self.node, message)
            self.set_model(tool="detacher", attached=False)

    def _call(self, client, request, timeout: float = 15.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        return client.call(request, timeout_sec=timeout)

    def _loaded_controllers(self) -> set[str]:
        from controller_manager_msgs.srv import ListControllers

        response = self._call(self._list, ListControllers.Request(), timeout=1.0)
        return {c.name for c in response.controller} if response else set()

    def _set_component(self, name: str, label: str) -> bool:
        from controller_manager_msgs.srv import SetHardwareComponentState
        from lifecycle_msgs.msg import State

        ids = {"active": State.PRIMARY_STATE_ACTIVE, "unconfigured": State.PRIMARY_STATE_UNCONFIGURED}
        request = SetHardwareComponentState.Request()
        request.name = name
        request.target_state = State(id=ids[label], label=label)
        response = self._call(self._set_hw, request)
        return bool(response and response.ok)

    def _switch_controllers(self, *, activate=(), deactivate=()) -> bool:
        from controller_manager_msgs.srv import SwitchController

        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.BEST_EFFORT
        response = self._call(self._switch, request)
        return bool(response and response.ok)

    def _deactivate(self, model: HandModel) -> None:
        if model.attached:
            self._switch_controllers(deactivate=[gripper_controller_name(model.tool), JOINT_STATE_BROADCASTER])
            self._set_component(model.component, "unconfigured")
        else:
            self._switch_controllers(deactivate=[JOINT_STATE_BROADCASTER])

    def _activate(self, model: HandModel) -> tuple[bool, str]:
        ok, message = True, ""
        if model.attached and not self._set_component(model.component, "active"):
            ok, message = False, f"{model.component} could not be activated (is the tool attached?)"
        controllers = [JOINT_STATE_BROADCASTER]
        if ok and model.attached:
            controllers.append(gripper_controller_name(model.tool))
        # joint_state_broadcaster takes the interfaces of the components active at activation
        self._switch_controllers(activate=controllers)
        return ok, message

    # -- ROS 1 hardware ------------------------------------------------------
    def _start_ros1_controllers(self, _event) -> None:
        from detachable_tools_common import ros1_hardware

        self._ros1_timer.shutdown()
        ros1_hardware.set_controller_params(self.model, self.include_detacher)
        if not ros1_hardware.start_controllers(self.model, self.include_detacher, self.hand_effort_limit):
            ros_compat.logerr(self.node, "failed to start the controllers")

    def _restart_ros1_hardware(self, model: HandModel) -> tuple[bool, str]:
        from detachable_tools_common import ros1_hardware

        if ros1_hardware.restart_hardware(model, self.include_detacher, self.hand_effort_limit):
            return True, ""
        detached = self._describe(tool="detacher", attached=False)
        ros1_hardware.restart_hardware(detached, self.include_detacher, self.hand_effort_limit)
        self.set_model(tool="detacher", attached=False)
        return False, f"{model.component} could not be started (is the tool attached?)"

    # -- model ---------------------------------------------------------------
    def _service(self, name, callback):
        if self.use_ros2_control:
            return self.node.create_service(Trigger, name, callback, callback_group=self._group)
        return ros_compat.create_service(self.node, name, Trigger, callback)

    def _describe(self, *, tool: str, attached: bool) -> HandModel:
        return describe_hand(self.side, tool, attached=attached, include_detacher=self.include_detacher, **self.hardware)

    def destroy_node(self) -> None:
        if ROS_VERSION == 2 and self.node is not None:
            self.node.destroy_node()

    def publish_model(self) -> None:
        stamp = ros_compat.stamp_now(self.node)
        if ROS_VERSION == 1:
            description = String()
            description.data = self.model.robot_description
            self.robot_description_pub.publish(description)
        self.tool_state_pub.publish(state_to_msg(self.model, stamp))
        self.hand_description_pub.publish(description_to_msg(self.model, stamp))
        ros_compat.loginfo(
            self.node,
            f"{self.side}: tool={self.model.tool}, attached={self.model.attached}, "
            f"motors={[m.id for m in self.model.motors]}",
        )

    def set_model(self, *, tool: str, attached: bool) -> HandModel:
        model = self._describe(tool=tool, attached=attached)
        if model.robot_description_sha256 == self.model.robot_description_sha256:
            return self.model
        self.model = model
        self.publish_model()
        self._update_robot_state_publisher(model)
        if ROS_VERSION == 1:
            # read by monitor_connection.py when it restarts dual_hand.launch
            ros_compat.set_param(self.node, "current_tool", model.tool)
        return model

    def set_tool(self, tool: str) -> tuple[bool, str]:
        if not self.include_detacher:
            return False, "Tool changing is disabled because the detacher is disabled"
        try:
            tool = normalize_tool(tool)
            attached = tool != "detacher"
            new_model = self._describe(tool=tool, attached=attached)
        except Exception as exc:
            return False, str(exc)
        if self.use_ros2_control and new_model.component != self.model.component:
            self._deactivate(self.model)
            ok, message = self._activate(new_model)
            if not ok:
                self.set_model(tool="detacher", attached=False)
                return False, message
        if self.restart_hardware and new_model.robot_description_sha256 != self.model.robot_description_sha256:
            ok, message = self._restart_ros1_hardware(new_model)
            if not ok:
                return False, message
        self.set_model(tool=tool, attached=attached)
        return True, state_to_text(self.model)

    def _make_set_tool_cb(self, tool: str):
        def _cb(_request, response):
            response.success, response.message = self.set_tool(tool)
            return response

        return _cb

    def _get_tool_state_cb(self, _request, response):
        response.success = True
        response.message = state_to_text(self.model)
        return response

    def _update_robot_state_publisher(self, model: HandModel) -> None:
        if ROS_VERSION == 1:
            # ROS 1 robot_state_publisher reads robot_description only at startup
            ros_compat.set_param(self.node, "robot_description", model.robot_description)
            return
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters

        client = self.node.create_client(
            SetParameters, f"{self.robot_state_publisher_node}/set_parameters", callback_group=self._group
        )
        request = SetParameters.Request()
        value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=model.robot_description)
        request.parameters = [Parameter(name="robot_description", value=value)]
        if self._call(client, request, timeout=2.0) is None:
            ros_compat.logwarn(self.node, f"{self.robot_state_publisher_node} is not available")
        self.node.destroy_client(client)


def _parse_bool(text: str | bool) -> bool:
    if isinstance(text, bool):
        return text
    return text.strip().lower() in ("1", "true", "yes", "on")


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=["lhand", "rhand"], required=True)
    parser.add_argument("--tool", default="generic")
    parser.add_argument("--attached", default="true")
    parser.add_argument("--detected-ids", default=None, help="startup discovery IDs, comma separated")
    parser.add_argument("--include-detacher", default="true")
    parser.add_argument("--port-name")
    parser.add_argument("--baud-rate")
    parser.add_argument("--protocol-version")
    parser.add_argument("--robot-state-publisher-node", default="robot_state_publisher")
    parser.add_argument("--controller-manager", default="controller_manager")
    parser.add_argument("--use-ros2-control", default="", help="default: true on ROS 2")
    parser.add_argument("--restart-hardware", default="false", help="ROS 1: restart dynamixel_general_control")
    parser.add_argument("--hand-effort-limit", type=float, default=0.92, help="ROS 1: hand Goal Current limit")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(node_argv(argv))
    node = ros_compat.init_node(f"{args.side}_modular_hand_manager")
    ModularHandManager(
        node=node,
        side=args.side,
        tool=args.tool,
        attached=_parse_bool(args.attached),
        include_detacher=_parse_bool(args.include_detacher),
        port_name=args.port_name,
        baud_rate=args.baud_rate,
        protocol_version=args.protocol_version,
        robot_state_publisher_node=args.robot_state_publisher_node,
        controller_manager=args.controller_manager,
        use_ros2_control=_parse_bool(args.use_ros2_control) if args.use_ros2_control else None,
        restart_hardware=_parse_bool(args.restart_hardware),
        hand_effort_limit=args.hand_effort_limit,
        detected_ids=[int(i) for i in args.detected_ids.split(",") if i] if args.detected_ids not in (None, "unspecified") else None,
    )
    ros_compat.spin(node, multi_threaded=True)


if __name__ == "__main__":
    main()
