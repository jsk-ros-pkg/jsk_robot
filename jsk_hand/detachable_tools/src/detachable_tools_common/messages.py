"""Tool state as diagnostic_msgs/DiagnosticArray (tool_state, hand_description) and YAML text (get_tool_state)."""

from __future__ import annotations

from dataclasses import fields

import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .description import DynamixelMotorSpec, HandModel

STATE_KEYS = (
    "side",
    "tool",
    "attached",
    "root_link",
    "tip_link",
    "command_joint",
    "detach_joint",
    "component",
    "robot_description_sha256",
)


def _value_text(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _key_values(items) -> list:
    values = []
    for key, value in items:
        kv = KeyValue()
        kv.key = key
        kv.value = _value_text(value)
        values.append(kv)
    return values


def state_dict(model: HandModel) -> dict:
    return {**{key: getattr(model, key) for key in STATE_KEYS},
            "motor_ids": [motor.id for motor in model.motors]}


def state_to_text(model: HandModel) -> str:
    data = state_dict(model)
    data["motors"] = {motor.name: motor.id for motor in model.motors}
    return yaml.safe_dump(data, default_flow_style=False, sort_keys=False)


def state_to_status(model: HandModel) -> DiagnosticStatus:
    status = DiagnosticStatus()
    status.name = f"{model.side}/tool_state"
    status.hardware_id = model.side
    status.message = model.tool if model.attached else "detached"
    status.values = _key_values(state_dict(model).items())
    return status


def motor_to_status(spec: DynamixelMotorSpec) -> DiagnosticStatus:
    status = DiagnosticStatus()
    status.name = f"{spec.side}/{spec.name}"
    status.hardware_id = f"dynamixel:{spec.id}"
    status.message = spec.role
    status.values = _key_values((field.name, getattr(spec, field.name)) for field in fields(spec))
    return status


def _array(stamp, statuses) -> DiagnosticArray:
    msg = DiagnosticArray()
    msg.header.stamp = stamp
    msg.status = list(statuses)
    return msg


def state_to_msg(model: HandModel, stamp) -> DiagnosticArray:
    return _array(stamp, [state_to_status(model)])


def description_to_msg(model: HandModel, stamp) -> DiagnosticArray:
    return _array(stamp, [state_to_status(model)] + [motor_to_status(motor) for motor in model.motors])


def status_to_dict(status: DiagnosticStatus) -> dict:
    return {kv.key: kv.value for kv in status.values}


def detection_to_msg(model, stamp, detected_ids, port_name):
    """Startup discovery snapshot, distinct from configured tool_state."""
    status = DiagnosticStatus()
    status.name = f"{model.side}/detected_tool"
    status.hardware_id = port_name
    status.message = model.tool if model.attached else "no tool detected"
    status.values = _key_values([
        ("side", model.side), ("port", port_name),
        ("detected_ids", sorted(detected_ids)), ("tool", model.tool),
        ("attached", model.attached), ("detection_scope", "startup"),
    ])
    return _array(stamp, [status])
