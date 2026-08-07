"""URDF-backed detachable hand model description helpers.

The source URDF files keep the visual/collision model and transmissions.  This
module renders the runtime ``robot_description`` by adding the ROS 2 control
hardware block and detachable-tool metadata.  Both controller_manager and
robot_state_publisher can then consume the same description.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Iterable, Mapping, Sequence
from xml.etree import ElementTree as ET

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - used only outside a sourced ROS env
    get_package_share_directory = None


PACKAGE_NAME = "dynamixel_detachable_hand"
VALID_SIDES = ("lhand", "rhand")
VALID_TOOLS = ("detacher", "generic", "needle_holder", "gripper", "forceps")

DEFAULT_BAUD_RATE = 57600
DEFAULT_PROTOCOL_VERSION = "2.0"
DEFAULT_TORQUE_CONSTANT = 1.15
DEFAULT_DYNAMIXEL_CURRENT_UNIT = DEFAULT_TORQUE_CONSTANT / 1000.0
DEFAULT_HAND_OPERATING_MODE = 5
DEFAULT_DETACH_OPERATING_MODE = 0
DEFAULT_HAND_EFFORT_LIMIT = 2.0
DEFAULT_DETACH_EFFORT_LIMIT = 2.0
DEFAULT_VELOCITY_LIMIT = 6.8
DEFAULT_FIXED_MOTOR_IDS = {
    "lhand": {
        "detacher": 0,
        "generic": 2,
        "gripper": 2,
        "needle_holder": 4,
        "forceps": 6,
    },
    "rhand": {
        "detacher": 1,
        "generic": 3,
        "gripper": 3,
        "needle_holder": 5,
        "forceps": 7,
    },
}

_TOOL_ALIASES = {
    "": "generic",
    "default": "generic",
    "dummy": "generic",
    "none": "detacher",
    "detached": "detacher",
    "detacher": "detacher",
    "generic": "generic",
    "needle-holder": "needle_holder",
    "needle_holder": "needle_holder",
    "holder": "needle_holder",
    "gripper": "gripper",
    "sesshi": "gripper",
    "setsushi": "gripper",
    "tweezers": "gripper",
    "forceps": "forceps",
    "scissors": "forceps",
    "scissor": "forceps",
    "hasami": "forceps",
}


@dataclass(frozen=True)
class DynamixelMotorSpec:
    side: str
    name: str
    joint: str
    id: int
    model: str
    operating_mode_id: int
    operating_mode: str
    torque_constant: float
    mechanical_reduction: float
    effort_limit: float
    velocity_limit: float
    current_limit: float
    calibration_offset: float
    role: str
    command_interface: str


@dataclass(frozen=True)
class HandModel:
    side: str
    tool: str
    attached: bool
    root_link: str
    tip_link: str
    command_joint: str
    detach_joint: str
    robot_description_sha256: str
    motors: tuple[DynamixelMotorSpec, ...]
    robot_description: str
    path: Path


@dataclass(frozen=True)
class ToolCoupling:
    driver: str
    joint: str
    poly: tuple[float, ...]


def package_share_path() -> Path:
    source_root = Path(__file__).resolve().parents[2]
    if (source_root / "config" / "tools.yaml").exists() and (source_root / "urdf").exists():
        return source_root
    if get_package_share_directory is not None:
        try:
            return Path(get_package_share_directory(PACKAGE_NAME))
        except Exception:
            pass
    return Path(__file__).resolve().parents[2]


def normalize_side(side: str) -> str:
    side = (side or "").strip().lower()
    if side not in VALID_SIDES:
        raise ValueError(f"unsupported hand side: {side!r}")
    return side


def normalize_tool(tool: str, *, attached: bool = True) -> str:
    if not attached:
        return "detacher"
    key = (tool or "").strip().lower().replace(" ", "_")
    normalized = _TOOL_ALIASES.get(key)
    if normalized is None or normalized not in VALID_TOOLS:
        raise ValueError(f"unsupported detachable tool: {tool!r}")
    return normalized


def robot_description_path(side: str, tool: str = "generic", *, attached: bool = True) -> Path:
    side = normalize_side(side)
    tool = normalize_tool(tool, attached=attached)
    model_tool = _model_tool(tool)
    name = f"{side}_detacher.urdf" if model_tool == "detacher" else f"{side}_{model_tool}.urdf"
    path = package_share_path() / "urdf" / name
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def tool_config_path(path: str | Path | None = None) -> Path:
    resolved = Path(path) if path else package_share_path() / "config" / "tools.yaml"
    if not resolved.exists():
        raise FileNotFoundError(resolved)
    return resolved


def load_tool_config(path: str | Path | None = None) -> dict:
    with tool_config_path(path).open() as stream:
        data = yaml.safe_load(stream) or {}
    return data


def load_robot_description(
    side: str,
    tool: str = "generic",
    *,
    attached: bool = True,
    include_detacher: bool = True,
    port_name: str | None = None,
    baud_rate: int | str | None = None,
    protocol_version: str | float | None = None,
    tool_config: Mapping | None = None,
    tool_config_file: str | Path | None = None,
) -> str:
    return build_robot_description(
        side,
        tool,
        attached=attached,
        include_detacher=include_detacher,
        port_name=port_name,
        baud_rate=baud_rate,
        protocol_version=protocol_version,
        tool_config=tool_config,
        tool_config_file=tool_config_file,
    )


def describe_hand(
    side: str,
    tool: str = "generic",
    *,
    attached: bool = True,
    include_detacher: bool = True,
    port_name: str | None = None,
    baud_rate: int | str | None = None,
    protocol_version: str | float | None = None,
    tool_config: Mapping | None = None,
    tool_config_file: str | Path | None = None,
) -> HandModel:
    path = robot_description_path(side, tool, attached=attached)
    robot_description = build_robot_description(
        side,
        tool,
        attached=attached,
        include_detacher=include_detacher,
        port_name=port_name,
        baud_rate=baud_rate,
        protocol_version=protocol_version,
        tool_config=tool_config,
        tool_config_file=tool_config_file,
    )
    return parse_hand_model(
        robot_description,
        path=path,
        expected_side=side,
        expected_tool=tool,
        expected_attached=attached,
    )


def build_robot_description(
    side: str,
    tool: str = "generic",
    *,
    attached: bool = True,
    include_detacher: bool = True,
    port_name: str | None = None,
    baud_rate: int | str | None = None,
    protocol_version: str | float | None = None,
    tool_config: Mapping | None = None,
    tool_config_file: str | Path | None = None,
) -> str:
    """Render a runtime URDF with ROS 2 control and detachable-tool metadata."""

    side = normalize_side(side)
    tool = normalize_tool(tool, attached=attached)
    config = dict(tool_config or load_tool_config(tool_config_file))
    path = robot_description_path(side, tool, attached=attached)
    root = ET.fromstring(path.read_text())

    _drop_generated_blocks(root)

    side_config = _side_config(config, side)
    port = port_name or side_config.get("default_port_name") or f"/dev/{side}"
    baud = str(baud_rate or _hardware_config(config).get("default_baud_rate", DEFAULT_BAUD_RATE))
    protocol = str(protocol_version or _hardware_config(config).get("protocol_version", DEFAULT_PROTOCOL_VERSION))

    motors = _runtime_motors(root, side, tool, attached, include_detacher, config)
    _inject_detachable_metadata(root, side, tool, attached, motors)
    _inject_couplings(root, side, tool, attached)
    _inject_ros2_control(root, side, port, baud, protocol, motors)
    _indent(root)
    return ET.tostring(root, encoding="unicode")


def resolve_tool_from_ids(
    side: str,
    present_ids: Iterable[int],
    *,
    tool_config: Mapping | None = None,
    tool_config_file: str | Path | None = None,
) -> tuple[str, bool]:
    """Return ``(tool, attached)`` for detected Dynamixel IDs."""

    side = normalize_side(side)
    config = dict(tool_config or load_tool_config(tool_config_file))
    ids = {int(value) for value in present_ids}
    tools = _side_config(config, side).get("tools", {})
    for tool, spec in tools.items():
        if not _bool_value(spec, "detectable", tool != "generic"):
            continue
        motor_id = _int_value(spec, "motor_id", None)
        if motor_id is not None and motor_id in ids:
            return normalize_tool(tool), True
    return "detacher", False


def scan_ids_from_config(side: str, *, tool_config: Mapping | None = None) -> tuple[int, ...]:
    side = normalize_side(side)
    config = dict(tool_config or load_tool_config())
    ids: set[int] = set()
    detacher_id = _int_value(_side_config(config, side).get("detacher", {}), "motor_id", None)
    if detacher_id is not None:
        ids.add(detacher_id)
    for tool, spec in _side_config(config, side).get("tools", {}).items():
        if not _bool_value(spec, "detectable", tool != "generic"):
            continue
        motor_id = _int_value(spec, "motor_id", None)
        if motor_id is not None:
            ids.add(motor_id)
    return tuple(sorted(ids))


def parse_hand_model(
    robot_description: str,
    *,
    path: Path | None = None,
    expected_side: str | None = None,
    expected_tool: str | None = None,
    expected_attached: bool | None = None,
) -> HandModel:
    root = ET.fromstring(robot_description)
    metadata = _metadata(root)
    side = normalize_side(expected_side or _attr(metadata, "side", ""))
    attached = _bool_attr(metadata, "attached", expected_attached if expected_attached is not None else True)
    tool = normalize_tool(expected_tool or _attr(metadata, "tool", "generic"), attached=attached)
    root_link = _attr(metadata, "root_link", f"{side}_base_link" if attached else "base_link")
    tip_link = _attr(metadata, "tip_link", _default_tip_link(side, tool, attached))
    command_joint = _attr(metadata, "command_joint", f"{side}_joint" if attached else "")
    detach_joint = _attr(metadata, "detach_joint", f"{side}_detach_joint_0")
    joints = _joint_names(root)
    links = _link_names(root)

    motors = tuple(_motor_specs(root, side))
    if not motors:
        raise ValueError("robot_description has no detachable_hand motor metadata")
    if len({m.id for m in motors}) != len(motors):
        raise ValueError(f"duplicate Dynamixel IDs in {path or '<robot_description>'}")
    for motor in motors:
        if motor.joint not in joints:
            raise ValueError(f"motor {motor.name} references missing joint {motor.joint}")
    if root_link not in links:
        raise ValueError(f"root_link {root_link} is missing from URDF")
    if tip_link and tip_link not in links:
        raise ValueError(f"tip_link {tip_link} is missing from URDF")
    if command_joint and command_joint not in joints:
        raise ValueError(f"command_joint {command_joint} is missing from URDF")
    if detach_joint not in joints:
        raise ValueError(f"detach_joint {detach_joint} is missing from URDF")

    return HandModel(
        side=side,
        tool=tool,
        attached=attached,
        root_link=root_link,
        tip_link=tip_link,
        command_joint=command_joint,
        detach_joint=detach_joint,
        robot_description_sha256=hashlib.sha256(robot_description.encode()).hexdigest(),
        motors=motors,
        robot_description=robot_description,
        path=path or Path("<robot_description>"),
    )


def extract_couplings(robot_description: str) -> tuple[ToolCoupling, ...]:
    root = ET.fromstring(robot_description)
    couplings: list[ToolCoupling] = []
    for element in root.iter():
        if _local_name(element.tag) != "coupling":
            continue
        driver = _attr(element, "driver", "")
        joint = _attr(element, "joint", "")
        poly = _float_list(_attr(element, "poly", ""))
        if not driver or not joint or not poly:
            raise ValueError(f"invalid coupling metadata: {ET.tostring(element, encoding='unicode')}")
        couplings.append(ToolCoupling(driver=driver, joint=joint, poly=tuple(poly)))
    return tuple(couplings)


def evaluate_polynomial(poly: Iterable[float], x_value: float) -> float:
    value = 0.0
    for coeff in poly:
        value = value * x_value + coeff
    return value


def _runtime_motors(
    root: ET.Element,
    side: str,
    tool: str,
    attached: bool,
    include_detacher: bool,
    config: Mapping,
) -> tuple[DynamixelMotorSpec, ...]:
    side_config = _side_config(config, side)
    motors: list[DynamixelMotorSpec] = []
    if attached:
        tool_spec = side_config.get("tools", {}).get(tool, {})
        motors.append(
            _make_motor_spec(
                root,
                side=side,
                name=f"{side}_motor",
                joint=f"{side}_joint",
                motor_id=_int_value(tool_spec, "motor_id", _default_motor_id(side, tool)),
                operating_mode_id=_int_value(tool_spec, "operating_mode", DEFAULT_HAND_OPERATING_MODE),
                operating_mode="current_based_position",
                role=f"tool:{tool}",
                command_interface="position",
            )
        )

    # A detached model must always retain its detacher motor.  For an attached
    # tool, callers may omit it when only the tool Dynamixel is physically
    # present (for example, the dual gripper setup using IDs 2 and 3).
    if include_detacher or not attached:
        detacher_spec = side_config.get("detacher", {})
        motors.append(
            _make_motor_spec(
                root,
                side=side,
                name=f"{side}_detach_motor_0",
                joint=f"{side}_detach_joint_0",
                motor_id=_int_value(detacher_spec, "motor_id", _default_motor_id(side, "detacher")),
                operating_mode_id=_int_value(detacher_spec, "operating_mode", DEFAULT_DETACH_OPERATING_MODE),
                operating_mode="current",
                role="detacher",
                command_interface="effort",
            )
        )
    return tuple(motors)


def _make_motor_spec(
    root: ET.Element,
    *,
    side: str,
    name: str,
    joint: str,
    motor_id: int,
    operating_mode_id: int,
    operating_mode: str,
    role: str,
    command_interface: str,
) -> DynamixelMotorSpec:
    limits = _joint_limits(root).get(joint, {})
    reductions = _mechanical_reductions(root)
    return DynamixelMotorSpec(
        side=side,
        name=name,
        joint=joint,
        id=motor_id,
        model="XC330-T288-T",
        operating_mode_id=operating_mode_id,
        operating_mode=operating_mode,
        torque_constant=DEFAULT_TORQUE_CONSTANT,
        mechanical_reduction=reductions.get(joint, 1.0),
        effort_limit=limits.get("effort", DEFAULT_HAND_EFFORT_LIMIT),
        velocity_limit=limits.get("velocity", DEFAULT_VELOCITY_LIMIT),
        current_limit=0.0,
        calibration_offset=0.0,
        role=role,
        command_interface=command_interface,
    )


def _inject_detachable_metadata(
    root: ET.Element,
    side: str,
    tool: str,
    attached: bool,
    motors: Sequence[DynamixelMotorSpec],
) -> None:
    metadata = ET.SubElement(
        root,
        "detachable_tool",
        {
            "side": side,
            "tool": tool,
            "attached": str(attached).lower(),
            "root_link": f"{side}_base_link",
            "tip_link": _default_tip_link(side, tool, attached),
            "command_joint": f"{side}_joint" if attached else "",
            "detach_joint": f"{side}_detach_joint_0",
        },
    )
    for motor in motors:
        ET.SubElement(
            metadata,
            "motor",
            {
                "side": motor.side,
                "name": motor.name,
                "joint": motor.joint,
                "id": str(motor.id),
                "model": motor.model,
                "operating_mode_id": str(motor.operating_mode_id),
                "operating_mode": motor.operating_mode,
                "torque_constant": str(motor.torque_constant),
                "mechanical_reduction": str(motor.mechanical_reduction),
                "effort_limit": str(motor.effort_limit),
                "velocity_limit": str(motor.velocity_limit),
                "current_limit": str(motor.current_limit),
                "calibration_offset": str(motor.calibration_offset),
                "role": motor.role,
                "command_interface": motor.command_interface,
            },
        )


def _inject_couplings(root: ET.Element, side: str, tool: str, attached: bool) -> None:
    if not attached or tool == "detacher":
        return
    couplings_file = package_share_path() / "config" / side / f"{tool}_loop_couplings.yaml"
    if not couplings_file.exists():
        return
    data = yaml.safe_load(couplings_file.read_text()) or {}
    metadata = _metadata(root)
    if metadata is None:
        return
    for group in data.get("couplings", []):
        driver = group.get("driver", "")
        for follower in group.get("followers", []):
            ET.SubElement(
                metadata,
                "coupling",
                {
                    "driver": str(driver),
                    "joint": str(follower.get("joint", "")),
                    "poly": " ".join(str(value) for value in follower.get("poly", [])),
                },
            )


def _inject_ros2_control(
    root: ET.Element,
    side: str,
    port_name: str,
    baud_rate: str,
    protocol_version: str,
    motors: Sequence[DynamixelMotorSpec],
) -> None:
    control = ET.SubElement(root, "ros2_control", {"name": f"{side}_dynamixel_hardware_interface", "type": "system"})
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "dynamixel_hardware_interface/DynamixelHardware"
    ET.SubElement(hardware, "param", {"name": "port_name"}).text = port_name
    ET.SubElement(hardware, "param", {"name": "baud_rate"}).text = baud_rate
    ET.SubElement(hardware, "param", {"name": "number_of_joints"}).text = str(len(motors))
    ET.SubElement(hardware, "param", {"name": "number_of_transmissions"}).text = str(len(motors))
    ET.SubElement(hardware, "param", {"name": "dynamixel_model_folder"}).text = "/param/dxl_model"
    ET.SubElement(hardware, "param", {"name": "disable_torque_at_init"}).text = "true"
    ET.SubElement(hardware, "param", {"name": "error_timeout_ms"}).text = "1000"
    ET.SubElement(hardware, "param", {"name": "dynamixel_state_pub_msg_name"}).text = (
        f"/{side}/dynamixel_hardware_interface/dxl_state"
    )
    ET.SubElement(hardware, "param", {"name": "get_dynamixel_data_srv_name"}).text = (
        f"/{side}/dynamixel_hardware_interface/get_dxl_data"
    )
    ET.SubElement(hardware, "param", {"name": "set_dynamixel_data_srv_name"}).text = (
        f"/{side}/dynamixel_hardware_interface/set_dxl_data"
    )
    ET.SubElement(hardware, "param", {"name": "reboot_dxl_srv_name"}).text = (
        f"/{side}/dynamixel_hardware_interface/reboot_dxl"
    )
    ET.SubElement(hardware, "param", {"name": "set_dxl_torque_srv_name"}).text = (
        f"/{side}/dynamixel_hardware_interface/set_dxl_torque"
    )
    ET.SubElement(hardware, "param", {"name": "protocol_version"}).text = protocol_version
    ET.SubElement(hardware, "param", {"name": "transmission_to_joint_matrix"}).text = _matrix_text(
        _transmission_to_joint_matrix(motors)
    )
    ET.SubElement(hardware, "param", {"name": "joint_to_transmission_matrix"}).text = _matrix_text(
        _joint_to_transmission_matrix(motors)
    )

    for motor in motors:
        gpio = ET.SubElement(control, "gpio", {"name": motor.name})
        ET.SubElement(gpio, "param", {"name": "ID"}).text = str(motor.id)
        ET.SubElement(gpio, "param", {"name": "type"}).text = "dxl"
        ET.SubElement(gpio, "param", {"name": "Return Delay Time"}).text = "0"
        ET.SubElement(gpio, "param", {"name": "Operating Mode"}).text = str(motor.operating_mode_id)
        ET.SubElement(gpio, "param", {"name": "Torque Enable"}).text = "1"
        ET.SubElement(gpio, "param", {"name": "[unit info]"}).text = _unit_info_text(motor)
        for interface in _dynamixel_command_interfaces(motor):
            ET.SubElement(gpio, "command_interface", {"name": interface})
        for interface in _dynamixel_state_interfaces():
            ET.SubElement(gpio, "state_interface", {"name": interface})

        joint = ET.SubElement(control, "joint", {"name": motor.joint})
        for interface in _joint_command_interfaces(motor):
            ET.SubElement(joint, "command_interface", {"name": interface})
        for interface in ("position", "velocity", "effort", "hardware_state", "torque_enable"):
            ET.SubElement(joint, "state_interface", {"name": interface})


def _dynamixel_command_interfaces(motor: DynamixelMotorSpec) -> tuple[str, ...]:
    if motor.command_interface == "position":
        return ("Goal Position", "Goal Current")
    if motor.command_interface == "effort":
        return ("Goal Current",)
    if motor.command_interface == "velocity":
        return ("Goal Velocity",)
    return (motor.command_interface,)


def _dynamixel_state_interfaces() -> tuple[str, ...]:
    return (
        "Present Position",
        "Present Velocity",
        "Present Current",
    )


def _joint_command_interfaces(motor: DynamixelMotorSpec) -> tuple[str, ...]:
    if motor.command_interface == "position":
        return ("position", "effort")
    if motor.command_interface == "effort":
        return ("effort",)
    return (motor.command_interface,)


def _unit_info_text(motor: DynamixelMotorSpec) -> str:
    current_unit = motor.torque_constant / 1000.0 if motor.torque_constant else DEFAULT_DYNAMIXEL_CURRENT_UNIT
    return (
        f"Present Current,{current_unit},N m,signed,0.0;"
        f"Goal Current,{current_unit},N m,signed,0.0"
    )


def _transmission_to_joint_matrix(motors: Sequence[DynamixelMotorSpec]) -> list[list[float]]:
    matrix: list[list[float]] = []
    for row, motor in enumerate(motors):
        matrix.append([])
        for column, _ in enumerate(motors):
            value = 0.0
            if row == column:
                reduction = motor.mechanical_reduction or 1.0
                value = 1.0 / reduction
            matrix[row].append(value)
    return matrix


def _joint_to_transmission_matrix(motors: Sequence[DynamixelMotorSpec]) -> list[list[float]]:
    matrix: list[list[float]] = []
    for row, motor in enumerate(motors):
        matrix.append([])
        for column, _ in enumerate(motors):
            value = motor.mechanical_reduction if row == column else 0.0
            matrix[row].append(value)
    return matrix


def _matrix_text(matrix: Sequence[Sequence[float]]) -> str:
    return ",".join(str(value) for row in matrix for value in row)


def _drop_generated_blocks(root: ET.Element) -> None:
    for child in list(root):
        if _local_name(child.tag) in ("detachable_tool", "ros2_control"):
            root.remove(child)


def _metadata(root: ET.Element) -> ET.Element | None:
    for element in root.iter():
        if _local_name(element.tag) == "detachable_tool":
            return element
    return None


def _motor_specs(root: ET.Element, side: str) -> Iterable[DynamixelMotorSpec]:
    reductions = _mechanical_reductions(root)
    limits = _joint_limits(root)
    for element in root.iter():
        if _local_name(element.tag) != "motor":
            continue
        joint = _attr(element, "joint", "")
        limit = limits.get(joint, {})
        yield DynamixelMotorSpec(
            side=_attr(element, "side", side),
            name=_attr(element, "name", ""),
            joint=joint,
            id=_int_attr(element, "id"),
            model=_attr(element, "model", "XC330-T288-T"),
            operating_mode_id=_int_attr(element, "operating_mode_id"),
            operating_mode=_attr(element, "operating_mode", ""),
            torque_constant=_float_attr(element, "torque_constant", DEFAULT_TORQUE_CONSTANT),
            mechanical_reduction=_float_attr(element, "mechanical_reduction", reductions.get(joint, 1.0)),
            effort_limit=_float_attr(element, "effort_limit", limit.get("effort", 0.0)),
            velocity_limit=_float_attr(element, "velocity_limit", limit.get("velocity", 0.0)),
            current_limit=_float_attr(element, "current_limit", 0.0),
            calibration_offset=_float_attr(element, "calibration_offset", 0.0),
            role=_attr(element, "role", "hand"),
            command_interface=_attr(element, "command_interface", "position"),
        )


def _side_config(config: Mapping, side: str) -> Mapping:
    return _hardware_config(config).get("sides", {}).get(side, {})


def _hardware_config(config: Mapping) -> Mapping:
    return config.get("hardware", {})


def _int_value(mapping: Mapping, key: str, default: int | None) -> int | None:
    value = mapping.get(key, default)
    return None if value is None else int(value)


def _bool_value(mapping: Mapping, key: str, default: bool) -> bool:
    value = mapping.get(key, default)
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _default_motor_id(side: str, tool: str) -> int:
    return DEFAULT_FIXED_MOTOR_IDS[side][normalize_tool(tool, attached=(tool != "detacher"))]


def _joint_limits(root: ET.Element) -> dict[str, dict[str, float]]:
    limits: dict[str, dict[str, float]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        limit = joint.find("limit")
        if not name or limit is None:
            continue
        limits[name] = {
            key: float(value)
            for key, value in limit.attrib.items()
            if key in ("effort", "velocity", "lower", "upper")
        }
    return limits


def _mechanical_reductions(root: ET.Element) -> dict[str, float]:
    reductions: dict[str, float] = {}
    for transmission in root.findall("transmission"):
        joint = transmission.find("joint")
        actuator = transmission.find("actuator")
        reduction = actuator.find("mechanicalReduction") if actuator is not None else None
        if joint is None or reduction is None or reduction.text is None:
            continue
        name = joint.attrib.get("name")
        if name:
            reductions[name] = float(reduction.text)
    return reductions


def _joint_names(root: ET.Element) -> set[str]:
    return {joint.attrib["name"] for joint in root.findall("joint") if "name" in joint.attrib}


def _link_names(root: ET.Element) -> set[str]:
    return {link.attrib["name"] for link in root.findall("link") if "name" in link.attrib}


def _default_tip_link(side: str, tool: str, attached: bool) -> str:
    if not attached or tool == "detacher":
        return f"{side}_detach_link_0"
    if tool == "needle_holder":
        return f"{side}_small_needle_holder_link2_1"
    if _model_tool(tool) == "gripper":
        return f"{side}_tip_link"
    return f"{side}_link"


def _model_tool(tool: str) -> str:
    return "gripper" if tool == "forceps" else tool


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1] if "}" in tag else tag


def _attr(element: ET.Element | None, key: str, default: str) -> str:
    if element is None:
        return default
    return element.attrib.get(key, default)


def _bool_attr(element: ET.Element | None, key: str, default: bool) -> bool:
    value = _attr(element, key, str(default)).strip().lower()
    return value in ("1", "true", "yes", "on")


def _int_attr(element: ET.Element, key: str) -> int:
    value = element.attrib.get(key)
    if value is None:
        raise ValueError(f"missing integer attribute {key} in {element.tag}")
    return int(value, 0)


def _float_attr(element: ET.Element, key: str, default: float) -> float:
    value = element.attrib.get(key)
    return default if value is None else float(value)


def _float_list(value: str) -> list[float]:
    return [float(item) for item in value.replace(",", " ").split()]


def _indent(element: ET.Element, level: int = 0) -> None:
    spacer = "\n" + level * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = spacer + "  "
        for child in element:
            _indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = spacer
    if level and (not element.tail or not element.tail.strip()):
        element.tail = spacer
