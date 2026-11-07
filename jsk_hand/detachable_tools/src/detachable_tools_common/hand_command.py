from __future__ import annotations

import argparse
import sys

from hand_controller import LHandController, RHandController
from hand_controller.hand_interface import ROS_VERSION, HandInterface


# command -> (tool, <group_state>) of srdf/<side>.srdf
NAMED = {
    "open": ("gripper", "open"),
    "close": ("gripper", "close"),
    "open-holder": ("needle_holder", "open"),
    "close-holder": ("needle_holder", "close"),
    "open-forceps": ("forceps", "open"),
    "close-forceps": ("forceps", "close"),
    "open-scissors": ("forceps", "open_scissors"),
    "close-scissors": ("forceps", "close_scissors"),
}


def _controller_class(side: str):
    return LHandController if side == "lhand" else RHandController


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=["lhand", "rhand"], required=True)
    parser.add_argument(
        "command",
        choices=sorted(NAMED) + ["attach", "detach"],
    )
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--current-ma", type=float)
    parser.add_argument("--effort", type=float)
    parser.add_argument("--dry-run", action="store_true")
    # drop ROS 1 remappings (__name:=...) and ROS 2 --ros-args
    if "--ros-args" in argv:
        argv = argv[: argv.index("--ros-args")]
    return parser.parse_args([arg for arg in argv if ":=" not in arg])


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
    from detachable_tools_common.description import named_position

    target = named_position(args.side, *NAMED[args.command])
    effort = HandInterface.HAND_EFFORT_LIMIT if args.effort is None else args.effort
    print(f"{args.side} {args.command}: joint={args.side}_joint target={target} effort={effort}")


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    if args.dry_run:
        _dry_run(args)
        return

    if ROS_VERSION == 2:
        import rclpy

        rclpy.init()
    else:
        import rospy

        rospy.init_node(f"{args.side}_hand_command", anonymous=True)
    controller = _controller_class(args.side)()
    try:
        if args.command in NAMED:
            method = getattr(controller, args.command.replace("-", "_"))
            method(tm=args.duration, effort=args.effort)
        elif args.command == "attach":
            current_ma = args.current_ma if args.current_ma is not None else -controller.DETACH_CURRENT_MA
            controller.command_detach_current(current_ma, tm=args.duration)
        elif args.command == "detach":
            current_ma = args.current_ma if args.current_ma is not None else controller.DETACH_CURRENT_MA
            controller.command_detach_current(current_ma, tm=args.duration)
    finally:
        controller.destroy()
        if ROS_VERSION == 2 and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
