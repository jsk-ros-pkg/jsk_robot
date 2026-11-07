#!/usr/bin/env python3
"""ROS 2 right-arm fork API. Dry-run by default; requires initial tag visibility.

Start from a user-positioned observation pose with the tag visible. All units SI.
"""
import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration as WaitDuration
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory
from detachable_tools_common.fork_simulation import RightArm, transform, fixed_transform, split_jsk_model
import xml.etree.ElementTree as ET
from detachable_tools_common.fork_planning import make_plan
from detachable_tools_common.fork_geometry import validate_seating
from detachable_tools_common.fork_vision import VisionGate


JOINTS = ['CHEST_JOINT0', 'HEAD_JOINT0', 'HEAD_JOINT1'] + [
    f'{side}_JOINT{i}' for side in ('LARM', 'RARM') for i in range(6)]


def matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    result = transform([t.x, t.y, t.z])
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return result


class ForkServer(Node):
    def __init__(self, config, hardware=False):
        super().__init__('nextage_fork_server')
        self.config = dict(config)
        self.hardware = hardware
        if hardware and config.get('calibrated') is not True:
            raise ValueError('Hardware mode requires calibrated: true and measured tag/fork/tool transforms')
        model = Path(get_package_share_directory('nextage_tutorials')) / 'urdf/JSKNextageOpenGripper.urdf'
        self.arm = RightArm(model.read_text())
        _, tool_xml, _, _ = split_jsk_model(model.read_text(),
            Path(get_package_share_directory('detachable_tools')))
        self.seating_report = validate_seating(tool_xml, config)
        wrist_tool = fixed_transform(ET.parse(model).getroot(), 'RARM_JOINT5_Link', 'rhand_base_link')
        fw = transform(config['fork_to_tool_xyz'], config['fork_to_tool_rpy']) @ np.linalg.inv(wrist_tool)
        self.config['fork_to_wrist_xyz'] = fw[:3, 3].tolist()
        self.config['fork_to_wrist_rpy'] = Rotation.from_matrix(fw[:3, :3]).as_euler('xyz').tolist()
        self.tag_to_fork = np.linalg.inv(transform(config['fork_to_tag_xyz'], config['fork_to_tag_rpy']))
        self.state = None
        self.busy = threading.Lock()
        self.io = ReentrantCallbackGroup()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(JointState, config.get('joint_states_topic', '/joint_states'),
                                 self.on_state, qos_profile_sensor_data, callback_group=self.io)
        self.action = None
        self.effort = None
        if hardware:
            self.action = ActionClient(self, FollowJointTrajectory, config['trajectory_action'], callback_group=self.io)
            self.effort = self.create_publisher(Float64MultiArray, config['effort_topic'], 1)
            self.controller_parameters = self.create_client(GetParameters,
                config['effort_controller'] + '/get_parameters', callback_group=self.io)
        self.fork_services = [self.create_service(Trigger, '/fork_exchange/' + name,
            self.callback(name), callback_group=self.io)
            for name in ('plan_detach', 'plan_attach', 'detach', 'attach')]
        self.get_logger().info('HARDWARE enabled' if hardware else 'DRY RUN: no action client or motor publisher')

    def on_state(self, msg):
        if len(msg.name) != len(msg.position):
            return
        values = dict(zip(msg.name, msg.position))
        if set(JOINTS) <= values.keys() and np.isfinite([values[n] for n in JOINTS]).all():
            self.state = (values, msg.header.stamp)

    def current(self):
        if self.state is None:
            raise RuntimeError('Waiting for all 15 Nextage joint positions')
        values, stamp = self.state
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        if not 0 <= age <= self.config.get('max_state_age', .5):
            raise RuntimeError('JointState is stale or has a future timestamp')
        return dict(values)

    def read_fork(self):
        detection = self.buffer.lookup_transform(self.config['camera_frame'], self.config['tag_frame'], Time())
        stamp = Time.from_msg(detection.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if not 0 <= age <= self.config.get('max_tag_age', .5):
            raise RuntimeError('AprilTag TF is stale or has a future timestamp')
        # RightArm FK is relative to CHEST, so chest yaw need not be zero.
        base_camera = self.buffer.lookup_transform('CHEST_JOINT0_Link', self.config['camera_frame'], stamp, timeout=WaitDuration(seconds=.5))
        fork = matrix(base_camera) @ matrix(detection) @ self.tag_to_fork
        return stamp.nanoseconds, fork

    def verify_vision(self, reference, expected_q):
        gate = VisionGate(self.get_clock().now().nanoseconds, reference, self.config)
        while rclpy.ok():
            measured = self.current()
            if np.max(np.abs(np.array([measured[n] for n in self.arm.names])-expected_q)) > .02:
                raise RuntimeError('Arm moved during the stationary image confirmation')
            try:
                stamp, pose = self.read_fork()
            except (TransformException, RuntimeError):
                stamp, pose = None, None
            result = gate.add(stamp, pose, self.get_clock().now().nanoseconds)
            if result is not None:
                return result
            time.sleep(.02)
        raise RuntimeError('ROS shutdown during image confirmation')

    def plan(self, operation):
        state = self.current()
        _, fork = self.read_fork()
        q = np.array([state[n] for n in self.arm.names])
        # Plan observation using the actual fixed camera TF (also in dry-run).
        wrist_camera = self.buffer.lookup_transform('RARM_JOINT5_Link',
            self.config['camera_frame'], Time(), timeout=WaitDuration(seconds=.5))
        planning_config = dict(self.config, wrist_to_camera=matrix(wrist_camera).tolist())
        frames, report = make_plan(self.arm, q, fork, planning_config, operation=operation)
        report.update(operation=operation, dry_run=not self.hardware,
                      fork_frame='CHEST_JOINT0_Link', fork_pose=fork.tolist(),
                      phases=list(dict.fromkeys(f[1] for f in frames)),
                      collision_checked=False, seating_geometry=self.seating_report)
        return state, frames, report

    def wait(self, future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done():
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise TimeoutError('ROS operation timed out')
            time.sleep(.01)
        return future.result()

    def preflight(self):
        if not self.action.wait_for_server(timeout_sec=3):
            raise RuntimeError('Nextage trajectory action unavailable')
        if not self.controller_parameters.wait_for_service(timeout_sec=3):
            raise RuntimeError('Detacher controller parameter service unavailable')
        req = GetParameters.Request(names=['joints'])
        response = self.wait(self.controller_parameters.call_async(req), 3)
        if len(response.values) != 1 or list(response.values[0].string_array_value) != ['rhand_detach_joint_0']:
            raise RuntimeError('Expected a single-joint right-hand detacher effort controller')
        if self.effort.get_subscription_count() == 0:
            raise RuntimeError('Detacher effort controller has no subscriber')

    def move(self, rows, held):
        current = self.current()
        if any(abs(current[n] - held[n]) > .03 for n in JOINTS if n not in self.arm.names):
            raise RuntimeError('Chest, head, or left arm moved during the exchange')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        dt = 1 / self.config['rate']
        # Explicit starting point avoids an unvalidated jump from the actual pose.
        qs = [np.array([current[n] for n in self.arm.names])] + [row[0] for row in rows]
        if np.max(np.abs(qs[1]-qs[0])) > .05:
            raise RuntimeError('Current right arm no longer matches the planned segment')
        for i, q in enumerate(qs):
            positions = dict(held)
            positions.update(zip(self.arm.names, q))
            ns = round(i * dt * 1e9)
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=[float(positions[n]) for n in JOINTS],
                time_from_start=Duration(sec=ns//10**9, nanosec=ns % 10**9)))
        future = self.action.send_goal_async(goal)
        try:
            handle = self.wait(future, 5)
        except BaseException:
            # A late acceptance must not leave an untracked trajectory running.
            future.add_done_callback(lambda f: f.result().cancel_goal_async() if f.result() and f.result().accepted else None)
            raise
        if not handle.accepted:
            raise RuntimeError('Nextage rejected the trajectory')
        try:
            result = self.wait(handle.get_result_async(), len(rows)*dt + 10)
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise RuntimeError('Trajectory failed: ' + result.result.error_string)
            if np.max(np.abs(np.array([self.current()[n] for n in self.arm.names])-qs[-1])) > .03:
                raise RuntimeError('Measured right arm did not reach the trajectory endpoint')
        except BaseException:
            self.wait(handle.cancel_goal_async(), 3)
            raise

    def latch(self, operation):
        # Same conversion/sign as HandInterface: +current releases, -current locks.
        effort = float(self.config['detach_current_ma']) / 1000 * 1.15
        if operation == 'ATTACH':
            effort = -effort
        try:
            end = time.monotonic() + self.config['latch_seconds']
            while time.monotonic() < end:
                if not rclpy.ok():
                    raise RuntimeError('ROS shutdown during latch pulse')
                self.effort.publish(Float64MultiArray(data=[effort]))
                time.sleep(.05)
        finally:
            self.effort.publish(Float64MultiArray(data=[0.]))

    def callback(self, name):
        def run(request, response):
            if not self.busy.acquire(blocking=False):
                response.success, response.message = False, 'Another fork operation is active'
                return response
            try:
                operation = name.removeprefix('plan_')
                held, frames, report = self.plan(operation)
                if self.hardware and not name.startswith('plan_'):
                    self.preflight()
                    from itertools import groupby
                    for phase, rows in groupby(frames, key=lambda f: f[1]):
                        rows = list(rows)
                        if phase.startswith('VERIFY_'):
                            check = self.verify_vision(np.array(report['fork_pose']), rows[-1][0])
                            check['phase'] = phase
                            report.setdefault('vision_checkpoints', []).append(check)
                        elif phase in ('ATTACH', 'DETACH'):
                            self.latch(phase)
                        else:
                            self.move(rows, held)
                    report['executed'] = True
                    report['latch_confirmation'] = 'timed current command only; no mechanical sensor'
                else:
                    report['executed'] = False
                response.success, response.message = True, json.dumps(report)
            except Exception as exc:
                response.success, response.message = False, str(exc)
                self.get_logger().error(str(exc))
            finally:
                self.busy.release()
            return response
        return run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--hardware', action='store_true')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    rclpy.init(args=[])
    server = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        server = ForkServer(config, args.hardware)
        executor.add_node(server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if server:
            server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
