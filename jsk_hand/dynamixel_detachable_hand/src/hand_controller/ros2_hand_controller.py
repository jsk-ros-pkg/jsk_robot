from __future__ import annotations

import time
from typing import TYPE_CHECKING
from typing import Iterable

if TYPE_CHECKING:
    from rclpy.node import Node


class HandInterface:
    DETACH_TORQUE_CONSTANT = 1.15
    DETACH_CURRENT_MA = 400.0
    HAND_EFFORT_LIMIT = 0.5

    def __init__(
        self,
        side: str,
        *,
        node: "Node | None" = None,
        groupname: str | None = None,
        create_action_client: bool = True,
        effort_joints: Iterable[str] | None = None,
    ) -> None:
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        self.side = side
        self.groupname = groupname or side
        self.node = node or rclpy.create_node(f"{side}_hand_controller")
        self.owns_node = node is None
        self.joint_name = f"{side}_joint"
        self.detach_joint_name = f"{side}_detach_joint_0"
        self.effort_joints = tuple(effort_joints or (self.detach_joint_name,))
        self.joint_states: dict[str, float] = {}
        self.detach_effort_pub = self.node.create_publisher(
            Float64MultiArray,
            f"/{self.groupname}/joint_group_effort_controller/commands",
            1,
        )
        self.node.create_subscription(
            JointState,
            f"/{self.groupname}/joint_states",
            self._joint_states_callback,
            10,
        )
        self.action_client = None
        if create_action_client:
            self.action_client = ActionClient(
                self.node,
                FollowJointTrajectory,
                f"/{self.groupname}/position_joint_trajectory_controller/follow_joint_trajectory",
            )

    def destroy(self) -> None:
        if self.owns_node:
            self.node.destroy_node()

    def _joint_states_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.joint_states[name] = position

    def get_joint_state(self, joint_name: str) -> float | None:
        return self.joint_states.get(joint_name)

    @classmethod
    def detach_effort_from_current(cls, current_ma: float) -> float:
        return (current_ma / 1000.0) * cls.DETACH_TORQUE_CONSTANT

    @classmethod
    def detach_effort_command_for_joints(
        cls,
        side: str,
        joints: Iterable[str],
        detach_effort: float,
        *,
        hand_effort_limit: float | None = None,
    ) -> list[float]:
        detach_joint = f"{side}_detach_joint_0"
        hand_limit = cls.HAND_EFFORT_LIMIT if hand_effort_limit is None else hand_effort_limit
        return [detach_effort if joint == detach_joint else hand_limit for joint in joints]

    def _resolve_effort_joints(self) -> tuple[str, ...]:
        import rclpy
        from rclpy.parameter_client import AsyncParameterClient

        client = AsyncParameterClient(self.node, f"/{self.groupname}/joint_group_effort_controller")
        try:
            if not client.services_are_ready() and not client.wait_for_services(timeout_sec=0.2):
                return self.effort_joints
            future = client.get_parameters(["joints"])
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
            response = future.result()
        except Exception:
            return self.effort_joints
        if response is None or not response.values:
            return self.effort_joints
        joints = tuple(response.values[0].string_array_value)
        if joints:
            self.effort_joints = joints
        return self.effort_joints

    def detach_effort_command(self, detach_effort: float) -> list[float]:
        return self.detach_effort_command_for_joints(self.side, self._resolve_effort_joints(), detach_effort)

    def set_detach_effort(self, detach_effort: float) -> list[float]:
        from std_msgs.msg import Float64MultiArray

        msg = Float64MultiArray()
        msg.data = self.detach_effort_command(detach_effort)
        self.detach_effort_pub.publish(msg)
        return list(msg.data)

    def stop_detach(self) -> list[float]:
        return self.set_detach_effort(0.0)

    def command_detach_current(self, current_ma: float, *, wait: bool = True, tm: float = 1.0) -> list[float]:
        command = self.set_detach_effort(self.detach_effort_from_current(current_ma))
        if tm is not None and tm > 0.0:
            if wait:
                time.sleep(tm)
                self.stop_detach()
            else:
                self.node.create_timer(tm, self.stop_detach)
        return command

    def move_hand(
        self,
        grasp_angle: float,
        *,
        wait: bool = True,
        tm: float = 1.0,
        velocity: float = 0.0,
        acceleration: float = 0.0,
        effort: float | None = None,
    ):
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint

        if self.action_client is None:
            raise RuntimeError("action client is disabled")
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("position_joint_trajectory_controller action server is not available")
        hand_effort = self.HAND_EFFORT_LIMIT if effort is None else effort
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.joint_name]
        point = JointTrajectoryPoint()
        point.positions = [grasp_angle]
        point.velocities = [velocity]
        point.accelerations = [acceleration]
        point.effort = [hand_effort]
        point.time_from_start.sec = int(tm)
        point.time_from_start.nanosec = int((tm - int(tm)) * 1_000_000_000)
        goal.trajectory.points = [point]
        send_future = self.action_client.send_goal_async(goal)
        if not wait:
            return send_future
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=tm + 5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("hand trajectory goal was rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=tm + 5.0)
        return result_future.result()

    def cancel_move_hand(self) -> None:
        if self.action_client is not None:
            self.action_client.server_is_ready()

    def open(
        self,
        *,
        wait: bool = True,
        tm: float = 1.0,
        velocity: float = 2.0,
        acceleration: float = 0.0,
        effort: float | None = None,
    ):
        return self.move_hand(0.0, wait=wait, tm=tm, velocity=velocity, acceleration=acceleration, effort=effort)

    def close(
        self,
        *,
        wait: bool = True,
        tm: float = 1.0,
        velocity: float = 2.0,
        acceleration: float = 0.0,
        effort: float | None = None,
    ):
        return self.move_hand(-2.7, wait=wait, tm=tm, velocity=velocity, acceleration=acceleration, effort=effort)

    def attach(self, *, wait: bool = True, tm: float = 1.0, **_) -> list[float]:
        return self.command_detach_current(-self.DETACH_CURRENT_MA, wait=wait, tm=tm)

    def detach(self, *, wait: bool = True, tm: float = 1.0, **_) -> list[float]:
        return self.command_detach_current(self.DETACH_CURRENT_MA, wait=wait, tm=tm)
