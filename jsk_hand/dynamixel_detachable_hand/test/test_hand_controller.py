from __future__ import annotations

from hand_controller import HandInterface


def test_detach_effort_command_for_attached_hand() -> None:
    data = HandInterface.detach_effort_command_for_joints(
        "lhand",
        ["lhand_joint", "lhand_detach_joint_0"],
        0.46,
    )

    assert data == [0.5, 0.46]


def test_detach_effort_command_for_detacher_only() -> None:
    data = HandInterface.detach_effort_command_for_joints(
        "rhand",
        ["rhand_detach_joint_0"],
        -0.46,
    )

    assert data == [-0.46]


def test_current_to_effort_conversion() -> None:
    assert HandInterface.detach_effort_from_current(400.0) == 0.45999999999999996


def test_default_effort_joints_are_detacher_only() -> None:
    import rclpy

    rclpy.init()
    interface = HandInterface("lhand", create_action_client=False)
    try:
        assert interface.effort_joints == ("lhand_detach_joint_0",)
    finally:
        interface.destroy()
        rclpy.shutdown()
