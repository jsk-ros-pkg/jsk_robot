from __future__ import annotations

import argparse
import sys

import yaml

from dynamixel_detachable_hand_common.description import (
    build_srdf,
    build_control_description,
    build_dynamixel_info,
    build_robot_description,
    describe_hand,
)


def _parse_bool(text: str | bool) -> bool:
    if isinstance(text, bool):
        return text
    return text.strip().lower() in ("1", "true", "yes", "on")


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=["lhand", "rhand"], required=True)
    parser.add_argument("--tool", default="generic")
    parser.add_argument("--attached", default="true")
    parser.add_argument("--include-detacher", default="true")
    parser.add_argument("--port-name")
    parser.add_argument("--baud-rate")
    parser.add_argument("--protocol-version")
    parser.add_argument("--robot-description-file", default="", help="tool model URDF (empty: default)")
    parser.add_argument("--summary", action="store_true")
    parser.add_argument("--control", action="store_true", help="ROS 2 controller_manager description (all tools)")
    parser.add_argument("--dynamixel-info", action="store_true", help="ROS 1 dynamixel_general_hw dynamixel_info")
    parser.add_argument("--srdf", action="store_true", help="named hand positions (srdf/<side>.srdf)")
    # roslaunch appends __name:=/__log:= to <param command=...>
    args, _unknown = parser.parse_known_args([arg for arg in argv if ":=" not in arg])
    return args


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    attached = _parse_bool(args.attached)
    include_detacher = _parse_bool(args.include_detacher)
    hardware = dict(port_name=args.port_name, baud_rate=args.baud_rate, protocol_version=args.protocol_version)
    if args.srdf:
        print(build_srdf(args.side))
    elif args.dynamixel_info:
        info = build_dynamixel_info(
            args.side,
            args.tool,
            attached=attached,
            include_detacher=include_detacher,
            source_urdf=args.robot_description_file or None,
        )
        print(yaml.safe_dump(info, default_flow_style=False, sort_keys=False), end="")
    elif args.control:
        print(build_control_description(args.side, include_detacher=include_detacher, **hardware))
    elif args.summary:
        model = describe_hand(args.side, args.tool, attached=attached, include_detacher=include_detacher, **hardware)
        motors = ",".join(f"{motor.joint}:{motor.id}" for motor in model.motors)
        print(f"{model.side} tool={model.tool} attached={model.attached} motors={motors}")
    else:
        print(
            build_robot_description(
                args.side,
                args.tool,
                attached=attached,
                include_detacher=include_detacher,
                source_urdf=args.robot_description_file or None,
                **hardware,
            )
        )


if __name__ == "__main__":
    main()
