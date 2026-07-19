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
