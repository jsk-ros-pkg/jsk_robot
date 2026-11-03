"""Hand descriptions from urdf/*.urdf (motors as <ros2_control>): display, control (ROS 2) and dynamixel_info (ROS 1)."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import os
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - ROS 1
    get_package_share_directory = None

try:
    import rospkg
except Exception:  # pragma: no cover - ROS 2
    rospkg = None


PACKAGE_NAME = "dynamixel_detachable_hand"
VALID_SIDES = ("lhand", "rhand")
TOOLS: tuple[str, ...] = ()  # the <group> names of srdf/lhand.srdf, set below
VALID_TOOLS: tuple[str, ...] = ()
DEFAULT_TORQUE_CONSTANT = 1.15

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
# tools drawn with another tool's model
_MODEL_TOOL = {"forceps": "gripper"}


@dataclass(frozen=True)
class DynamixelMotorSpec:
    side: str
    name: str
    joint: str
    id: int
    operating_mode_id: int
    component: str
    role: str


@dataclass(frozen=True)
class HandModel:
    side: str
    tool: str
    attached: bool
    root_link: str
    tip_link: str
    command_joint: str
    detach_joint: str
    component: str
    robot_description_sha256: str
    motors: tuple[DynamixelMotorSpec, ...]
    robot_description: str


@dataclass(frozen=True)
class ToolCoupling:
    driver: str
    joint: str
    poly: tuple[float, ...]


def use_tool_prefix() -> bool:
    """ROS 2 names the tool joints ``<side>_<tool>_joint``; ROS 1 keeps ``<side>_joint``."""
    return os.environ.get("ROS_VERSION", "1") == "2"


def package_share_path() -> Path:
    source_root = Path(__file__).resolve().parents[2]
    if (source_root / "urdf").exists():
        return source_root
    if get_package_share_directory is not None:
        try:
            return Path(get_package_share_directory(PACKAGE_NAME))
        except Exception:
            pass
    if rospkg is not None:
        try:
            return Path(rospkg.RosPack().get_path(PACKAGE_NAME))
        except Exception:
            pass
    return source_root


def normalize_side(side: str) -> str:
    side = (side or "").strip().lower()
    if side not in VALID_SIDES:
        raise ValueError(f"unsupported hand side: {side!r}")
    return side


def normalize_tool(tool: str, *, attached: bool = True) -> str:
    if not attached:
        return "detacher"
    key = (tool or "").strip().lower().replace(" ", "_").replace("-", "_")
    normalized = _TOOL_ALIASES.get(key, key if key in TOOLS else None)
    if normalized is None:
        raise ValueError(f"unsupported detachable tool: {tool!r} (tools: {', '.join(TOOLS)})")
    return normalized


def urdf_path(side: str, tool: str) -> Path:
    path = package_share_path() / "urdf" / f"{side}_{tool}.urdf"
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def robot_description_path(side: str, tool: str = "generic", *, attached: bool = True) -> Path:
    """Source URDF of the tool model."""
    side = normalize_side(side)
    tool = normalize_tool(tool, attached=attached)
    return urdf_path(side, _MODEL_TOOL.get(tool, tool))


def component_name(side: str, tool: str) -> str:
    return f"{side}_{tool}"


def hand_joint_name(side: str, tool: str, *, prefix: bool | None = None) -> str:
    prefix = use_tool_prefix() if prefix is None else prefix
    return f"{side}_{tool}_joint" if prefix and tool != "detacher" else f"{side}_joint"


def gripper_controller_name(tool: str) -> str:
    return f"{tool}_gripper_controller"


def build_robot_description(
    side: str,
    tool: str = "generic",
    *,
    attached: bool = True,
    include_detacher: bool = True,
    port_name: str | None = None,
    baud_rate: int | str | None = None,
    protocol_version: str | float | None = None,
    source_urdf: str | Path | None = None,
    prefix: bool | None = None,
) -> str:
    """Display description: the detacher and the attached tool."""
    side = normalize_side(side)
    tool = normalize_tool(tool, attached=attached)
    prefix = use_tool_prefix() if prefix is None else prefix
    root = _detacher_root(side, include_detacher=include_detacher or not attached)
    if attached:
        _merge_tool(root, side, tool, prefix=prefix, source_urdf=source_urdf)
    _set_hardware_params(root, port_name, baud_rate, protocol_version)
    _inject_metadata(root, side, tool, attached, prefix)
    _indent(root)
    return ET.tostring(root, encoding="unicode")


def build_control_description(
    side: str,
    *,
    include_detacher: bool = True,
    port_name: str | None = None,
    baud_rate: int | str | None = None,
    protocol_version: str | float | None = None,
) -> str:
    """Control description for ``controller_manager``: the detacher and every tool."""
    side = normalize_side(side)
    root = _detacher_root(side, include_detacher=include_detacher)
    root.set("name", f"{side}_detachable_hand")
    for tool in TOOLS:
        _merge_tool(root, side, tool, prefix=True)
    _set_hardware_params(root, port_name, baud_rate, protocol_version)
    _indent(root)
    return ET.tostring(root, encoding="unicode")


def srdf_path(side: str) -> Path:
    path = package_share_path() / "srdf" / f"{normalize_side(side)}.srdf"
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def named_positions(side: str) -> dict[str, dict[str, float]]:
    """``{tool: {state: motor angle}}`` from the <group_state> of srdf/<side>.srdf."""
    side = normalize_side(side)
    positions: dict[str, dict[str, float]] = {}
    for state in ET.parse(srdf_path(side)).getroot().iter("group_state"):
        tool = state.get("group", "")[len(side) + 1:]
        positions.setdefault(tool, {})[state.get("name")] = float(state.find("joint").get("value"))
    return positions


def named_position(side: str, tool: str, state: str) -> float:
    return named_positions(side)[normalize_tool(tool)][state]


def srdf_tools(side: str = "lhand") -> tuple[str, ...]:
    """Tools of the hand: the <group> names of srdf/<side>.srdf without the side prefix."""
    side = normalize_side(side)
    return tuple(g.get("name")[len(side) + 1:] for g in ET.parse(srdf_path(side)).getroot().iter("group"))


def build_srdf(side: str, *, prefix: bool | None = None) -> str:
    """srdf/<side>.srdf with the joint names of the description (ROS 2: <side>_<tool>_joint)."""
    side = normalize_side(side)
    prefix = use_tool_prefix() if prefix is None else prefix
    root = ET.parse(srdf_path(side)).getroot()
    if prefix:
        for element in root:
            tool = element.get("group" if element.tag == "group_state" else "name", "")[len(side) + 1:]
            for joint in element.iter("joint"):
                joint.set("name", hand_joint_name(side, tool, prefix=True))
    _indent(root)
    return ET.tostring(root, encoding="unicode")


def build_dynamixel_info(
    side: str,
    tool: str = "generic",
    *,
    attached: bool = True,
    include_detacher: bool = True,
    source_urdf: str | Path | None = None,
) -> dict:
    """ROS 1 ``dynamixel_general_hw`` ``dynamixel_info`` from the ``<ros2_control>`` blocks."""
    side = normalize_side(side)
    tool = normalize_tool(tool, attached=attached)
    motors = []
    if attached:
        root = ET.parse(source_urdf or urdf_path(side, tool)).getroot()
        motors += _motor_specs(root, side, tool)
    if include_detacher or not attached:
        motors += _motor_specs(ET.parse(urdf_path(side, "detacher")).getroot(), side, "detacher")
    return {
        motor.name: {
            "ID": motor.id,
            "Return_Delay_Time": 0,
            "Operating_Mode": motor.operating_mode_id,
            "torque_constant": DEFAULT_TORQUE_CONSTANT,
        }
        for motor in motors
    }


def describe_hand(
    side: str,
    tool: str = "generic",
    *,
    attached: bool = True,
    include_detacher: bool = True,
    port_name: str | None = None,
    baud_rate: int | str | None = None,
    protocol_version: str | float | None = None,
    prefix: bool | None = None,
) -> HandModel:
    description = build_robot_description(
        side,
        tool,
        attached=attached,
        include_detacher=include_detacher,
        port_name=port_name,
        baud_rate=baud_rate,
        protocol_version=protocol_version,
        prefix=prefix,
    )
    return parse_hand_model(description)


def parse_hand_model(robot_description: str) -> HandModel:
    root = ET.fromstring(robot_description)
    meta = _metadata(root)
    if meta is None:
        raise ValueError("robot_description has no detachable_tool metadata")
    side = normalize_side(meta.get("side", ""))
    attached = meta.get("attached", "true") == "true"
    tool = normalize_tool(meta.get("tool", "generic"), attached=attached)
    motors = []
    for control in root.findall("ros2_control"):
        comp_tool = control.get("name", "")[len(side) + 1:]
        motors += _motor_specs(control, side, comp_tool)
    return HandModel(
        side=side,
        tool=tool,
        attached=attached,
        root_link=meta.get("root_link", ""),
        tip_link=meta.get("tip_link", ""),
        command_joint=meta.get("command_joint", ""),
        detach_joint=meta.get("detach_joint", ""),
        component=meta.get("component", ""),
        robot_description_sha256=hashlib.sha256(robot_description.encode()).hexdigest(),
        motors=tuple(motors),
        robot_description=robot_description,
    )


def extract_couplings(robot_description: str) -> tuple[ToolCoupling, ...]:
    root = ET.fromstring(robot_description)
    couplings = []
    for element in root.iter("coupling"):
        poly = tuple(float(v) for v in element.get("poly", "").replace(",", " ").split())
        if not element.get("driver") or not element.get("joint") or not poly:
            raise ValueError(f"invalid coupling metadata: {ET.tostring(element, encoding='unicode')}")
        couplings.append(ToolCoupling(driver=element.get("driver"), joint=element.get("joint"), poly=poly))
    return tuple(couplings)


def evaluate_polynomial(poly: Iterable[float], x_value: float) -> float:
    value = 0.0
    for coeff in poly:
        value = value * x_value + coeff
    return value


# ---------------------------------------------------------------------------


def _detacher_names(side: str) -> set[str]:
    root = ET.parse(urdf_path(side, "detacher")).getroot()
    names = {e.get("name") for e in root if e.tag in ("link", "joint", "transmission")}
    names |= {e.get("name") for e in root.iter() if e.tag in ("actuator", "gpio")}
    return names


def _detacher_root(side: str, *, include_detacher: bool) -> ET.Element:
    root = ET.parse(urdf_path(side, "detacher")).getroot()
    if not include_detacher:
        for control in root.findall("ros2_control"):
            root.remove(control)
    return root


def _merge_tool(root: ET.Element, side: str, tool: str, *, prefix: bool, source_urdf=None) -> None:
    """Append the tool parts (not in the detacher URDF) of ``tool`` to ``root``."""
    shared = _detacher_names(side)
    model_root = ET.parse(source_urdf or robot_description_path(side, tool)).getroot()
    control_root = ET.parse(urdf_path(side, tool)).getroot()
    rename = _renamer(side, tool, shared) if prefix else (lambda name: name)
    for element in model_root:
        if element.tag in ("ros2_control", "detachable_tool"):
            continue
        if element.get("name") in shared:
            continue
        root.append(_renamed(element, rename))
    for control in control_root.findall("ros2_control"):
        root.append(_renamed(control, rename))


def _renamer(side: str, tool: str, shared: set[str]):
    head = f"{side}_"

    def rename(name: str) -> str:
        if not name or name in shared or not name.startswith(head):
            return name
        return f"{side}_{tool}_{name[len(head):]}"

    return rename


def _renamed(element: ET.Element, rename) -> ET.Element:
    copy = ET.fromstring(ET.tostring(element))
    for e in copy.iter():
        if e.tag in ("link", "joint", "transmission", "actuator", "gpio", "parent", "child", "mimic"):
            for key in ("name", "link", "joint"):
                if key in e.attrib:
                    e.set(key, rename(e.get(key)))
    return copy


def _set_hardware_params(root, port_name, baud_rate, protocol_version) -> None:
    values = {"port_name": port_name, "baud_rate": baud_rate, "protocol_version": protocol_version}
    for param in root.iter("param"):
        value = values.get(param.get("name"))
        if value is not None and value != "":
            param.text = str(value)


def _motor_specs(root: ET.Element, side: str, tool: str) -> list[DynamixelMotorSpec]:
    controls = [root] if root.tag == "ros2_control" else root.findall("ros2_control")
    specs = []
    for control in controls:
        joints = [j.get("name") for j in control.findall("joint")]
        for gpio, joint in zip(control.findall("gpio"), joints):
            params = {p.get("name"): (p.text or "").strip() for p in gpio.findall("param")}
            specs.append(
                DynamixelMotorSpec(
                    side=side,
                    name=gpio.get("name"),
                    joint=joint,
                    id=int(params["ID"]),
                    operating_mode_id=int(params.get("Operating Mode", "0")),
                    component=control.get("name", ""),
                    role="detacher" if tool == "detacher" else f"tool:{tool}",
                )
            )
    return specs


def _inject_metadata(root: ET.Element, side: str, tool: str, attached: bool, prefix: bool) -> None:
    joint_names = {j.get("name") for j in root.findall("joint")}
    link_names = {link.get("name") for link in root.findall("link")}
    tip = _default_tip_link(side, tool, attached)
    if prefix and attached:
        tip = tip.replace(f"{side}_", f"{side}_{tool}_", 1)
    meta = ET.SubElement(
        root,
        "detachable_tool",
        {
            "side": side,
            "tool": tool,
            "attached": str(attached).lower(),
            "root_link": f"{side}_base_link",
            "tip_link": tip if tip in link_names else f"{side}_base_link",
            "command_joint": hand_joint_name(side, tool, prefix=prefix) if attached else "",
            "detach_joint": f"{side}_detach_joint_0",
            "component": component_name(side, tool) if attached else "",
        },
    )
    if not attached:
        return
    couplings = package_share_path() / "config" / side / f"{_MODEL_TOOL.get(tool, tool)}_loop_couplings.yaml"
    if not couplings.exists():
        return
    rename = _renamer(side, tool, _detacher_names(side)) if prefix else (lambda name: name)
    for group in (yaml.safe_load(couplings.read_text()) or {}).get("couplings", []):
        for follower in group.get("followers", []):
            joint = rename(str(follower.get("joint", "")))
            driver = rename(str(group.get("driver", "")))
            if joint in joint_names and driver in joint_names:
                ET.SubElement(
                    meta,
                    "coupling",
                    {"driver": driver, "joint": joint, "poly": " ".join(str(v) for v in follower.get("poly", []))},
                )


def _default_tip_link(side: str, tool: str, attached: bool) -> str:
    if not attached or tool == "detacher":
        return f"{side}_detach_link_0"
    if tool == "needle_holder":
        return f"{side}_small_needle_holder_link2_1"
    if _MODEL_TOOL.get(tool, tool) == "gripper":
        return f"{side}_tip_link"
    return f"{side}_link"


def _metadata(root: ET.Element) -> ET.Element | None:
    return root.find("detachable_tool")


def _indent(element: ET.Element, level: int = 0) -> None:
    ET.indent(element, space="  ", level=level)


TOOLS = srdf_tools("lhand")
VALID_TOOLS = ("detacher",) + TOOLS
