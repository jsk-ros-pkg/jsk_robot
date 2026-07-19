from __future__ import annotations

import argparse
import sys

from hand_controller import LHandController, RHandController
from hand_controller.ros2_hand_controller import HandInterface


def _controller_class(side: str):
    return LHandController if side == "lhand" else RHandController


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=["lhand", "rhand"], required=True)
    parser.add_argument(
        "command",
        choices=["open", "close", "open-holder", "close-holder", "attach", "detach"],
    )
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--current-ma", type=float)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def _dry_run(args: argparse.Namespace) -> None:
    joints = [f"{args.side}_joint", f"{args.side}_detach_joint_0"]
    if args.command in ("attach", "detach"):
        current_ma = args.current_ma
        if current_ma is None:
            current_ma = -HandInterface.DETACH_CURRENT_MA if args.command == "attach" else HandInterface.DETACH_CURRENT_MA
        effort = HandInterface.detach_effort_from_current(current_ma)
        data = HandInterface.detach_effort_command_for_joints(args.side, joints, effort)
        print(f"{args.side} {args.command}: current_ma={current_ma:.1f} effort_command={data}")
        return
    target = {
        "open": 0.0,
        "close": -2.7,
        "open-holder": -0.1,
        "close-holder": 0.08,
    }[args.command]
    print(f"{args.side} {args.command}: joint={args.side}_joint target={target}")


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    if args.dry_run:
        _dry_run(args)
        return

    import rclpy

    rclpy.init()
    controller = _controller_class(args.side)()
    try:
        if args.command == "open":
            controller.open(tm=args.duration)
        elif args.command == "close":
            controller.close(tm=args.duration)
        elif args.command == "open-holder":
            controller.open_holder(tm=args.duration)
        elif args.command == "close-holder":
            controller.close_holder(tm=args.duration)
        elif args.command == "attach":
            current_ma = args.current_ma if args.current_ma is not None else -controller.DETACH_CURRENT_MA
            controller.command_detach_current(current_ma, tm=args.duration)
        elif args.command == "detach":
            current_ma = args.current_ma if args.current_ma is not None else controller.DETACH_CURRENT_MA
            controller.command_detach_current(current_ma, tm=args.duration)
    finally:
        controller.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
