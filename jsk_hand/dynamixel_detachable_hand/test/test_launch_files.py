from __future__ import annotations

from pathlib import Path

import pytest

Parser = pytest.importorskip("launch_xml").Parser


@pytest.mark.parametrize(
    "launch_file",
    [
        "hand_model.launch.xml",
        "hand_control.launch.xml",
        "hand_detacher.launch.xml",
        "dual_hand.launch.xml",
        "dual_detacher.launch.xml",
    ],
)
def test_ros2_xml_launch_files_parse(launch_file: str) -> None:
    root, parser = Parser.load(Path("launch") / launch_file)
    description = parser.parse_description(root)

    assert description.entities


def test_dual_hand_launch_uses_only_tool_motor_controllers() -> None:
    launch_text = Path("launch/dual_hand.launch.xml").read_text()

    assert launch_text.count('<arg name="include_detacher" value="false" />') == 2
    assert "config/lhand/tool_only_controllers.yaml" in launch_text
    assert "config/rhand/tool_only_controllers.yaml" in launch_text
