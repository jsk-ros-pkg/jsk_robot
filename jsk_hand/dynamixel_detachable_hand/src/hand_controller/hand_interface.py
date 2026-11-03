"""Detachable hand client shared by ROS 1 (dynamixel_general_hw) and ROS 2 (ros2_control)."""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING
from typing import Iterable


from dynamixel_detachable_hand_common.ros_compat import ROS_VERSION, ros_compat

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
        self.side = side
        self.groupname = groupname or side
        self.ros_version = ROS_VERSION
        self.joint_name = f"{side}_joint"
        self.detach_joint_name = f"{side}_detach_joint_0"
        self.effort_joints = tuple(effort_joints or (self.detach_joint_name,))
        self.hand_effort = self.HAND_EFFORT_LIMIT
        self.joint_states: dict[str, float] = {}
        self.action_client = None
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self.node = None
        self.owns_node = False
        if ROS_VERSION == 2:
            self._init_ros2(node, create_action_client)
        else:
            self._init_ros1(create_action_client)

    # -- setup --
    def _init_ros1(self, create_action_client: bool) -> None:
        import rospy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        if not rospy.core.is_initialized():
            rospy.init_node(f"{self.side}_hand_controller", anonymous=True)
        # Goal Current limit of the hand motor set by hand_control.launch
        self.hand_effort = float(
            rospy.get_param(f"/{self.groupname}/hand_effort_limit", self.HAND_EFFORT_LIMIT)
        )
        self.detach_effort_pub = ros_compat.create_publisher(None, self.effort_command_topic, Float64MultiArray, 1)
        self._joint_states_sub = ros_compat.create_subscription(
            None, f"/{self.groupname}/joint_states", JointState, self._joint_states_callback, 10
        )
        if create_action_client:
            import actionlib
            from control_msgs.msg import FollowJointTrajectoryAction

            self.action_client = actionlib.SimpleActionClient(self.trajectory_action_name, FollowJointTrajectoryAction)

    def _init_ros2(self, node, create_action_client: bool) -> None:
        import rclpy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        if node is None:
            if not rclpy.ok():
                rclpy.init()
            node = rclpy.create_node(f"{self.side}_hand_controller")
            self.owns_node = True
        self.node = node
        self.detach_effort_pub = self.node.create_publisher(Float64MultiArray, self.effort_command_topic, 1)
        self.node.create_subscription(
            JointState,
            f"/{self.groupname}/joint_states",
            self._joint_states_callback,
            10,
        )
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.qos import DurabilityPolicy, QoSProfile

        self.tool = None
        self.node.create_subscription(
            DiagnosticArray,
            f"/{self.groupname}/tool_state",
            self._tool_state_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        if create_action_client:
            # one ParallelGripperCommand client per tool controller
            self.action_client = {}

    def _tool_state_callback(self, msg) -> None:
        for status in msg.status:
            values = {kv.key: kv.value for kv in status.values}
            if "tool" in values:
                attached = values.get("attached") == "true"
                self.tool = values["tool"] if attached else None
                self.joint_name = values.get("command_joint") or f"{self.side}_joint"

    def _gripper_client(self):
        """Action client of the attached tool's gripper controller."""
        from control_msgs.action import ParallelGripperCommand
        from rclpy.action import ActionClient

        deadline = time.time() + 2.0
        while self.tool is None and time.time() < deadline:
            self._spin_once(0.05)
        if self.tool is None:
            raise RuntimeError(f"no tool is attached to {self.groupname}")
        if self.tool not in self.action_client:
            name = f"/{self.groupname}/{self.tool}_gripper_controller/gripper_cmd"
            self.action_client[self.tool] = ActionClient(self.node, ParallelGripperCommand, name)
        return self.action_client[self.tool]

    @property
    def trajectory_action_name(self) -> str:
        return f"/{self.groupname}/position_joint_trajectory_controller/follow_joint_trajectory"

    @property
    def effort_command_topic(self) -> str:
        # ros_control: command, ros2_control: commands
        suffix = "commands" if ROS_VERSION == 2 else "command"
        return f"/{self.groupname}/joint_group_effort_controller/{suffix}"

    def destroy(self) -> None:
        if ROS_VERSION == 2:
            if self.owns_node and self.node is not None:
                self.node.destroy_node()
        else:
            self.detach_effort_pub.unregister()
            self._joint_states_sub.unregister()

    # -- helpers --
    def _sleep(self, seconds: float) -> None:
        if ROS_VERSION == 2:
            time.sleep(seconds)
        else:
            import rospy

            rospy.sleep(seconds)

    def _spin_once(self, timeout_sec: float = 0.01) -> None:
        if ROS_VERSION == 2:
            import rclpy

            rclpy.spin_once(self.node, timeout_sec=timeout_sec)
        else:
            self._sleep(timeout_sec)

    def _joint_states_callback(self, msg) -> None:
        for name, position in zip(msg.name, msg.position):
            self.joint_states[name] = position
            if name == self.joint_name:
                # ROS 2 tool joints are <side>_<tool>_joint; keep <side>_joint available
                self.joint_states[f"{self.side}_joint"] = position

    def get_joint_state(self, joint_name: str) -> float | None:
        if ROS_VERSION == 2:
            self._spin_once(0.0)
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
        if ROS_VERSION == 2:
            return self._resolve_effort_joints_ros2()
        import rospy

        joints = rospy.get_param(f"/{self.groupname}/joint_group_effort_controller/joints", None)
        if joints:
            self.effort_joints = tuple(joints)
        return self.effort_joints

    def _resolve_effort_joints_ros2(self) -> tuple[str, ...]:
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

    def _wait_for_effort_subscriber(self, timeout: float = 1.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if ROS_VERSION == 2:
                count = self.detach_effort_pub.get_subscription_count()
            else:
                count = self.detach_effort_pub.get_num_connections()
            if count > 0:
                return True
            self._spin_once(0.05)
        return False

    # -- detacher (attach / detach) --
    def detach_effort_command(self, detach_effort: float) -> list[float]:
        return self.detach_effort_command_for_joints(
            self.side,
            self._resolve_effort_joints(),
            detach_effort,
            hand_effort_limit=self.hand_effort,
        )

    def set_detach_effort(self, detach_effort: float) -> list[float]:
        from std_msgs.msg import Float64MultiArray

        self._wait_for_effort_subscriber()
        msg = Float64MultiArray()
        msg.data = self.detach_effort_command(detach_effort)
        self.detach_effort_pub.publish(msg)
        return list(msg.data)

    def stop_detach(self) -> list[float]:
        return self.set_detach_effort(0.0)

    def command_detach_current(self, current_ma: float, wait: bool = True, tm: float = 1.0) -> list[float]:
        command = self.set_detach_effort(self.detach_effort_from_current(current_ma))
        if tm is not None and tm > 0.0:
            if wait:
                self._sleep(tm)
                self.stop_detach()
            elif ROS_VERSION == 2:
                timer = None

                def _stop() -> None:
                    timer.cancel()
                    self.stop_detach()

                timer = self.node.create_timer(tm, _stop)
            else:
                threading.Timer(tm, self.stop_detach).start()
        return command

    def move_detach_joint(
        self,
        angle: float = 0.0,
        wait: bool = True,
        tm: float = 1.0,
        velocity: float = 0.3,
        acceleration: float = 0.0,
        effort: float = 0.0,
    ):
        current_ma = effort / self.DETACH_TORQUE_CONSTANT * 1000.0
        return self.command_detach_current(current_ma, wait=wait, tm=tm)

    def attach(self, wait: bool = True, tm: float = 1.0, velocity=0.3, acceleration=0.0, effort=None) -> list[float]:
        return self.command_detach_current(-self.DETACH_CURRENT_MA, wait=wait, tm=tm)

    def detach(self, wait: bool = True, tm: float = 1.0, velocity=0.3, acceleration=0.0, effort=None) -> list[float]:
        return self.command_detach_current(self.DETACH_CURRENT_MA, wait=wait, tm=tm)

    # -- hand (open / close) --
    def set_hand_effort(self, effort: float) -> None:
        """Grasp current limit [Nm] (ROS 1: hand joint of joint_group_effort_controller)."""

        self.hand_effort = float(effort)
        if ROS_VERSION == 1 and self.joint_name in self._resolve_effort_joints():
            self.set_detach_effort(0.0)

    def _make_goal(self, goal, grasp_angle, tm, velocity, acceleration, effort) -> None:
        from trajectory_msgs.msg import JointTrajectoryPoint

        goal.trajectory.joint_names = [self.joint_name]
        point = JointTrajectoryPoint()
        point.positions = [float(grasp_angle)]
        point.velocities = [float(velocity)]
        point.accelerations = [float(acceleration)]
        point.effort = [float(effort)]
        if ROS_VERSION == 2:
            point.time_from_start.sec = int(tm)
            point.time_from_start.nanosec = int((tm - int(tm)) * 1_000_000_000)
        else:
            import rospy

            point.time_from_start = rospy.Duration.from_sec(tm)
        goal.trajectory.points = [point]

    def move_hand(
        self,
        grasp_angle: float,
        wait: bool = True,
        tm: float = 1.0,
        velocity: float = 0.0,
        acceleration: float = 0.0,
        effort: float | None = None,
    ):
        if self.action_client is None:
            raise RuntimeError("action client is disabled")
        # ROS 1 ignores the trajectory effort as before; use set_hand_effort() there
        hand_effort = self.hand_effort if effort is None else effort
        if ROS_VERSION == 2:
            return self._move_hand_ros2(grasp_angle, wait, tm, velocity, acceleration, hand_effort)
        return self._move_hand_ros1(grasp_angle, wait, tm, velocity, acceleration, hand_effort)

    def _move_hand_ros1(self, grasp_angle, wait, tm, velocity, acceleration, effort):
        import rospy
        from control_msgs.msg import FollowJointTrajectoryGoal

        if not self.action_client.wait_for_server(rospy.Duration(5.0)):
            raise RuntimeError("position_joint_trajectory_controller action server is not available")
        goal = FollowJointTrajectoryGoal()
        self._make_goal(goal, grasp_angle, tm, velocity, acceleration, effort)
        self.action_client.send_goal(goal)
        if not wait:
            return None
        # ROS 1 returns the actionlib GoalStatus, like the former ROS 1 interface
        from actionlib_msgs.msg import GoalStatus

        self.action_client.wait_for_result(rospy.Duration(tm + 5.0))
        state = self.action_client.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logwarn("Hand movement failed: {}".format(state))
        return state

    def _move_hand_ros2(self, grasp_angle, wait, tm, velocity, acceleration, effort):
        import rclpy
        from control_msgs.action import ParallelGripperCommand

        client = self._gripper_client()
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"{self.tool}_gripper_controller action server is not available")
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [self.joint_name]
        goal.command.position = [float(grasp_angle)]
        # max effort: Goal Current of the tool motor
        goal.command.effort = [float(effort)]
        self._goal_handle = None
        self._result_future = None
        self._send_future = client.send_goal_async(goal)
        if not wait:
            return self._send_future
        rclpy.spin_until_future_complete(self.node, self._send_future, timeout_sec=tm + 5.0)
        goal_handle = self._send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("hand gripper goal was rejected")
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, self._result_future, timeout_sec=tm + 5.0)
        return self._result_future.result()

    def _ros2_goal_handle(self):
        if self._goal_handle is None and self._send_future is not None:
            self._spin_once(0.0)
            if self._send_future.done():
                self._goal_handle = self._send_future.result()
        return self._goal_handle

    def cancel_move_hand(self) -> None:
        if self.action_client is None:
            return
        if ROS_VERSION == 2:
            goal_handle = self._ros2_goal_handle()
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
        else:
            self.action_client.cancel_goal()

    def hand_moving_p(self) -> bool:
        if self.action_client is None:
            return False
        if ROS_VERSION == 2:
            if self._send_future is not None and not self._send_future.done():
                return True
            goal_handle = self._ros2_goal_handle()
            if goal_handle is None or not goal_handle.accepted:
                return False
            if self._result_future is None:
                self._result_future = goal_handle.get_result_async()
            self._spin_once(0.0)
            return not self._result_future.done()
        from actionlib_msgs.msg import GoalStatus

        return self.action_client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE)

    def wait_for_hand(self, timeout: float | None = None) -> bool:
        deadline = None if timeout is None else time.time() + timeout
        while self.hand_moving_p():
            if deadline is not None and time.time() >= deadline:
                return False
            self._spin_once(0.01)
        return True

    def move_named(self, state: str, tool: str = "gripper", wait=True, tm=1.0, velocity=0.0, acceleration=0.0, effort=None):
        """Move to the <group_state> ``state`` of ``tool`` in srdf/<side>.srdf."""
        from dynamixel_detachable_hand_common.description import named_position

        position = named_position(self.side, tool, state)
        return self.move_hand(position, wait=wait, tm=tm, velocity=velocity, acceleration=acceleration, effort=effort)

    def open(self, wait=True, tm=1.0, velocity=2.0, acceleration=0.0, effort=None):
        return self.move_named("open", "gripper", wait, tm, velocity, acceleration, effort)

    def close(self, wait=True, tm=1.0, velocity=2.0, acceleration=0.0, effort=None):
        return self.move_named("close", "gripper", wait, tm, velocity, acceleration, effort)

    def open_holder(self, wait=True, tm=1.0, velocity=0.5, acceleration=0.0, effort=None):
        return self.move_named("open", "needle_holder", wait, tm, velocity, acceleration, effort)

    def close_holder(self, wait=True, tm=1.0, velocity=0.5, acceleration=0.0, effort=None):
        return self.move_named("close", "needle_holder", wait, tm, velocity, acceleration, effort)

    def open_forceps(self, wait=True, tm=1.0, velocity=1.0, acceleration=0.0, effort=None):
        return self.move_named("open", "forceps", wait, tm, velocity, acceleration, effort)

    def close_forceps(self, wait=True, tm=1.0, velocity=2.5, acceleration=0.0, effort=None):
        return self.move_named("close", "forceps", wait, tm, velocity, acceleration, effort)

    def open_scissors(self, wait=True, tm=1.0, velocity=3.0, acceleration=0.0, effort=None):
        return self.move_named("open_scissors", "forceps", wait, tm, velocity, acceleration, effort)

    def close_scissors(self, wait=True, tm=1.0, velocity=2.0, acceleration=0.0, effort=None):
        return self.move_named("close_scissors", "forceps", wait, tm, velocity, acceleration, effort)
