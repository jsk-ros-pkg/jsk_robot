from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("dynamixel_detachable_hand.srv")

from dynamixel_detachable_hand_common.modular_hand_manager import ModularHandManager


def test_simulated_id_detection_switches_robot_description() -> None:
    rclpy.init()
    node = None
    try:
        node = ModularHandManager(
            side="lhand",
            tool="detacher",
            attached=False,
            auto_detect=False,
            simulated_present_ids=[0, 2],
        )
        initial_sha = node.model.robot_description_sha256

        node.detect_and_update()

        assert node.model.tool == "gripper"
        assert node.model.attached is True
        assert node.model.robot_description_sha256 != initial_sha
        assert [motor.id for motor in node.model.motors] == [2, 0]
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
