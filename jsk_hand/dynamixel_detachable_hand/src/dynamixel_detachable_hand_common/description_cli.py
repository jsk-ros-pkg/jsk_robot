from __future__ import annotations

import argparse
import sys

from dynamixel_detachable_hand_common.description import (
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
    parser.add_argument("--tool-config-file")
    parser.add_argument("--summary", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    attached = _parse_bool(args.attached)
    include_detacher = _parse_bool(args.include_detacher)
    if args.summary:
        model = describe_hand(
            args.side,
            args.tool,
            attached=attached,
            include_detacher=include_detacher,
            port_name=args.port_name,
            baud_rate=args.baud_rate,
            protocol_version=args.protocol_version,
            tool_config_file=args.tool_config_file,
        )
        motors = ",".join(f"{motor.joint}:{motor.id}" for motor in model.motors)
        print(f"{model.side} tool={model.tool} attached={model.attached} motors={motors}")
        return
    print(
        build_robot_description(
            args.side,
            args.tool,
            attached=attached,
            include_detacher=include_detacher,
            port_name=args.port_name,
            baud_rate=args.baud_rate,
            protocol_version=args.protocol_version,
            tool_config_file=args.tool_config_file,
        )
    )


if __name__ == "__main__":
    main()
