from __future__ import annotations

from dynamixel_detachable_hand.msg import DynamixelMotor, HandDescription, ToolState

from .description import DynamixelMotorSpec, HandModel


def motor_to_msg(spec: DynamixelMotorSpec) -> DynamixelMotor:
    msg = DynamixelMotor()
    msg.side = spec.side
    msg.name = spec.name
    msg.joint = spec.joint
    msg.id = spec.id
    msg.model = spec.model
    msg.operating_mode_id = spec.operating_mode_id
    msg.operating_mode = spec.operating_mode
    msg.torque_constant = spec.torque_constant
    msg.mechanical_reduction = spec.mechanical_reduction
    msg.effort_limit = spec.effort_limit
    msg.velocity_limit = spec.velocity_limit
    msg.current_limit = spec.current_limit
    msg.calibration_offset = spec.calibration_offset
    msg.role = spec.role
    msg.command_interface = spec.command_interface
    return msg


def state_to_msg(model: HandModel, stamp) -> ToolState:
    msg = ToolState()
    msg.stamp = stamp
    msg.side = model.side
    msg.tool = model.tool
    msg.attached = model.attached
    msg.root_link = model.root_link
    msg.tip_link = model.tip_link
    msg.command_joint = model.command_joint
    msg.detach_joint = model.detach_joint
    msg.robot_description_sha256 = model.robot_description_sha256
    return msg


def description_to_msg(model: HandModel, stamp) -> HandDescription:
    msg = HandDescription()
    msg.stamp = stamp
    msg.side = model.side
    msg.tool = model.tool
    msg.attached = model.attached
    msg.root_link = model.root_link
    msg.tip_link = model.tip_link
    msg.command_joint = model.command_joint
    msg.detach_joint = model.detach_joint
    msg.robot_description_sha256 = model.robot_description_sha256
    msg.motors = [motor_to_msg(motor) for motor in model.motors]
    return msg
