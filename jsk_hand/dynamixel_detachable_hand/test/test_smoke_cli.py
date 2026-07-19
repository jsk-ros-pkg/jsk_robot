from __future__ import annotations

import subprocess
import sys


def run_cli(*args: str) -> str:
    completed = subprocess.run(
        [sys.executable, *args],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return completed.stdout


def test_print_robot_description_summary_smoke() -> None:
    output = run_cli(
        "scripts/print_robot_description.py",
        "--side",
        "lhand",
        "--tool",
        "generic",
        "--summary",
    )

    assert "lhand tool=generic attached=True" in output
    assert "lhand_joint:2" in output
    assert "lhand_detach_joint_0:0" in output


def test_detach_dry_run_smoke() -> None:
    output = run_cli("scripts/hand_command.py", "--side", "lhand", "detach", "--dry-run")

    assert "lhand detach" in output
    assert "effort_command=[0.92, 0.45999999999999996]" in output
