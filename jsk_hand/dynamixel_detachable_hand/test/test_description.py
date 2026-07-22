from __future__ import annotations

import pytest

from dynamixel_detachable_hand_common.description import (
    VALID_SIDES,
    build_robot_description,
    describe_hand,
    extract_couplings,
    load_tool_config,
    resolve_tool_from_ids,
    scan_ids_from_config,
)


EXPECTED_TOOL_IDS = {
    "lhand": {"generic": 2, "gripper": 2, "needle_holder": 4, "forceps": 6},
    "rhand": {"generic": 3, "gripper": 3, "needle_holder": 5, "forceps": 7},
}
EXPECTED_DETACHER_IDS = {"lhand": 0, "rhand": 1}


@pytest.mark.parametrize("side", VALID_SIDES)
@pytest.mark.parametrize("tool", ["generic", "needle_holder", "gripper", "forceps"])
def test_attached_robot_description_has_ros2_control(side: str, tool: str) -> None:
    model = describe_hand(side, tool, port_name=f"/dev/{side}")

    assert model.side == side
    assert model.tool == tool
    assert model.attached is True
    assert len(model.motors) == 2
    assert [motor.id for motor in model.motors] == [EXPECTED_TOOL_IDS[side][tool], EXPECTED_DETACHER_IDS[side]]
    assert f"<ros2_control name=\"{side}_dynamixel_general_hw\" type=\"system\">" in model.robot_description
    assert "dynamixel_general_hw/DynamixelGeneralHw" in model.robot_description
    assert f"<joint name=\"{side}_detach_joint_0\"" in model.robot_description


@pytest.mark.parametrize("side", VALID_SIDES)
def test_detacher_robot_description_has_only_detacher_motor(side: str) -> None:
    model = describe_hand(side, "detacher", attached=False)

    assert model.tool == "detacher"
    assert model.attached is False
    assert [motor.joint for motor in model.motors] == [f"{side}_detach_joint_0"]
    assert [motor.id for motor in model.motors] == [EXPECTED_DETACHER_IDS[side]]
    assert f"<param name=\"Operating_Mode\">0</param>" in model.robot_description


def test_needle_holder_couplings_are_embedded() -> None:
    robot_description = build_robot_description("lhand", "needle_holder")
    couplings = extract_couplings(robot_description)

    assert len(couplings) == 2
    assert {coupling.driver for coupling in couplings} == {"lhand_joint"}
    assert {coupling.joint for coupling in couplings} == {
        "lhand_holder_jaw_joint",
        "lhand_holder_tip_joint",
    }


def test_resolve_tool_from_present_ids() -> None:
    config = load_tool_config()

    assert resolve_tool_from_ids("lhand", [0], tool_config=config) == ("detacher", False)
    assert resolve_tool_from_ids("lhand", [0, 2], tool_config=config) == ("gripper", True)
    assert resolve_tool_from_ids("lhand", [0, 4], tool_config=config) == ("needle_holder", True)
    assert resolve_tool_from_ids("lhand", [0, 6], tool_config=config) == ("forceps", True)
    assert resolve_tool_from_ids("rhand", [5, 1], tool_config=config) == ("needle_holder", True)


def test_scan_ids_follow_fixed_side_offset() -> None:
    config = load_tool_config()

    assert scan_ids_from_config("lhand", tool_config=config) == (0, 2, 4, 6)
    assert scan_ids_from_config("rhand", tool_config=config) == (1, 3, 5, 7)
