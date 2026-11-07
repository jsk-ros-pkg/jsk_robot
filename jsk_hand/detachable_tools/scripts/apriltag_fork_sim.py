#!/usr/bin/env python3
"""Run an isolated ROS 2 / RViz AprilTag camera + Nextage fork simulation.

No controller, action client, or hardware command topic is created. See --help.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sysconfig
import tempfile
import time

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from detachable_tools_common.fork_simulation import (
    RightArm, TagCamera, make_plan, pose_error, transform, split_jsk_model)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--once', action='store_true', help='Exit after one detach/attach cycle')
    parser.add_argument('--domain-id', type=int, default=87, help='Isolated simulation ROS domain')
    parser.add_argument('--extra-prefix', type=Path, help='Optional extracted ROS dependency prefix')
    parser.add_argument('--output', type=Path, default=Path('/tmp/apriltag_fork_sim'))
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    if args.extra_prefix:
        prefix = str(args.extra_prefix.resolve())
        for key, suffix in [('AMENT_PREFIX_PATH', ''), ('LD_LIBRARY_PATH', '/lib')]:
            os.environ[key] = prefix + suffix + ':' + os.environ.get(key, '')
        multiarch = sysconfig.get_config_var('MULTIARCH')
        os.environ['LD_LIBRARY_PATH'] = prefix + '/lib/' + multiarch + ':' + os.environ['LD_LIBRARY_PATH']
    # ROS is imported only after selecting the isolated domain.
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import Image, CameraInfo, JointState
    from geometry_msgs.msg import TransformStamped, Point
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    import cv2

    source_share = Path(__file__).resolve().parent.parent
    share = source_share if (source_share / 'config/fork_simulation.yaml').exists() else Path(
        get_package_share_directory('detachable_tools'))
    config = yaml.safe_load((args.config or share / 'config/fork_simulation.yaml').read_text())
    description = Path(get_package_share_directory('nextage_tutorials')) / 'urdf/JSKNextageOpenGripper.urdf'
    urdf, tool_urdf, wrist_to_tool, wrist_to_camera = split_jsk_model(
        description.read_text(), share)
    arm = RightArm(urdf)
    fork_truth = transform(config['fork_xyz'], config['fork_rpy'])
    fork_to_tag = transform(config['fork_to_tag_xyz'], config['fork_to_tag_rpy'])
    tag_truth = fork_truth @ fork_to_tag
    fork_to_tool = transform(config['fork_to_tool_xyz'], config['fork_to_tool_rpy'])
    fork_to_wrist = fork_to_tool @ np.linalg.inv(wrist_to_tool)
    config['fork_to_wrist_xyz'] = fork_to_wrist[:3, 3].tolist()
    config['fork_to_wrist_rpy'] = Rotation.from_matrix(fork_to_wrist[:3, :3]).as_euler('xyz').tolist()
    initial = fork_truth @ fork_to_wrist
    initial[:3, 3] += fork_truth[:3, :3] @ config['approach_offset']
    q0 = arm.ik(initial, [-.28, -.20, -1.86, .27, .40, 1.19])
    camera = TagCamera(config)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'report.json').write_text(json.dumps({'completed': False}) + '\n')
    children = []
    logs = []

    def spawn(package, executable, extra):
        binary = Path(get_package_prefix(package)) / 'lib' / package / executable
        log = (args.output / (str(len(children)) + '_' + executable + '.log')).open('w')
        logs.append(log)
        children.append(subprocess.Popen([str(binary), *extra], stdout=log, stderr=log))

    class Simulation(Node):
        def __init__(self):
            super().__init__('apriltag_fork_sim')
            self.q = q0.copy()
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.broadcaster = TransformBroadcaster(self)
            self.joints = self.create_publisher(JointState, 'joint_states', 10)
            self.images = self.create_publisher(Image, 'wrist_camera/image_rect', 10)
            self.info = self.create_publisher(CameraInfo, 'wrist_camera/camera_info', 10)
            self.markers = self.create_publisher(MarkerArray, 'fork_sim/markers', 10)
            self.history = {}
            self.observations = []
            self.last_detection = None
            self.plan = None
            self.index = 0
            self.phase = 'WAIT_FOR_IMAGE_DETECTION'
            self.attached = True
            self.fork_estimate = None
            self.parked_tool = None
            self.report = {}
            self.done = False
            self.started = time.monotonic()
            self.previous_phase = None
            self.timer = self.create_timer(1/config['rate'], self.tick)

        def tf(self, parent, child, matrix, stamp):
            msg = TransformStamped()
            msg.header.frame_id, msg.child_frame_id = parent, child
            msg.header.stamp = stamp
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, matrix[:3, 3])
            quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
            msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, quat)
            return msg

        def detect(self):
            try:
                detected = self.buffer.lookup_transform('wrist_camera_optical', 'fork_tag', Time())
            except TransformException:
                return
            stamp = detected.header.stamp
            key = stamp.sec*10**9 + stamp.nanosec
            if key == self.last_detection or key not in self.history:
                return
            if (self.get_clock().now().nanoseconds - key) / 1e9 > .5:
                return
            self.last_detection = key
            tr, rot = detected.transform.translation, detected.transform.rotation
            camera_tag = transform([tr.x, tr.y, tr.z])
            camera_tag[:3, :3] = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
            # Use camera pose at the image timestamp, never the latest wrist pose.
            estimate = self.history[key] @ camera_tag @ np.linalg.inv(fork_to_tag)
            self.observations.append(estimate)
            if len(self.observations) < 10:
                return
            estimate[:3, 3] = np.mean([p[:3, 3] for p in self.observations], axis=0)
            estimate[:3, :3] = Rotation.from_matrix([p[:3, :3] for p in self.observations]).mean().as_matrix()
            pe, re = pose_error(estimate, fork_truth)
            # Ground truth is used only as a simulation assertion, not as a motion target.
            if pe > .01 or re > np.deg2rad(5):
                raise RuntimeError(f'Perception assertion failed: {pe:.4f} m / {np.rad2deg(re):.2f} deg')
            self.fork_estimate = estimate
            self.plan, self.report = make_plan(arm, self.q, estimate, config)
            self.report.update(perception_position_error_m=float(pe),
                               perception_rotation_error_deg=float(np.rad2deg(re)),
                               detector_samples=len(self.observations),
                               robot_model=str(description),
                               wrist_to_tool=wrist_to_tool.tolist(),
                               wrist_to_camera=wrist_to_camera.tolist(),
                               tag_mount='fork rear bar',
                               fork_estimate=estimate.tolist(),
                               camera_model='ideal pinhole, no occlusion or lighting',
                               collision_checked=False, physics_simulated=False)
            self.get_logger().info(f'Detected fork: error {pe*1000:.2f} mm / {np.rad2deg(re):.2f} deg; validated {len(self.plan)} IK samples')

        def draw(self, wrist, cam, stamp):
            markers = []
            def add(kind, matrix, scale, color, text=''):
                m = Marker()
                m.header.frame_id, m.header.stamp = 'WAIST', stamp
                m.ns, m.id, m.type, m.action = 'fork_sim', len(markers), kind, Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, matrix[:3, 3])
                quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
                m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, quat)
                m.scale.x, m.scale.y, m.scale.z = map(float, scale)
                m.color.r, m.color.g, m.color.b, m.color.a = map(float, color)
                m.text = text
                markers.append(m)
                return m
            # U-shaped rack opens toward fork -Y. Tool collar rests on the two tines.
            for x in [-.035, .035]:
                add(Marker.CUBE, fork_truth @ transform([x,-.045,0]), [.025,.15,.012], [.45,.5,.55,1])
            add(Marker.CUBE, fork_truth @ transform([0,.04,0]), [.095,.025,.012], [.45,.5,.55,1])
            add(Marker.CUBE, fork_truth @ transform([0,.04,-.12]), [.035,.035,.24], [.35,.4,.45,1])
            # Tag-bearing rear bar is rigidly connected to the fork post.
            tag_pos = np.array(config['fork_to_tag_xyz'])
            add(Marker.CUBE, fork_truth @ transform([tag_pos[0]/2, tag_pos[1]+.012, tag_pos[2]]),
                [abs(tag_pos[0])+.10,.02,.10], [.35,.4,.45,1])
            add(Marker.CUBE, fork_truth @ transform([0,(tag_pos[1]+.04)/2,tag_pos[2]]),
                [.025,abs(tag_pos[1]-.04)+.025,.025], [.35,.4,.45,1])
            # Actual tag bitmap as individual flat squares; white quiet border.
            cell = config['tag_size'] / 8
            add(Marker.CUBE, tag_truth @ transform([0,0,-.001]), [10*cell,10*cell,.002], [1,1,1,1])
            bitmap = cv2.resize(camera.bitmap, (8,8), interpolation=cv2.INTER_NEAREST)
            for row in range(8):
                for col in range(8):
                    if bitmap[row,col] < 128:
                        add(Marker.CUBE, tag_truth @ transform([(col-3.5)*cell,(3.5-row)*cell,.0005]),
                            [cell,cell,.001], [0,0,0,1])
            add(Marker.TEXT_VIEW_FACING, transform([.35,-.30,.52]), [0,0,.027], [1,1,1,1],
                'JSK_NEJINEJI\n' + self.phase + ('\nLOCKED' if self.attached else '\nRELEASED'))
            if self.plan:
                line = add(Marker.LINE_STRIP, np.eye(4), [.003,0,0], [.2,1,.3,1])
                for q, _, _ in self.plan:
                    p = arm.fk(q)[:3,3]
                    line.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            self.markers.publish(MarkerArray(markers=markers))

        def tick(self):
            if any(p.poll() is not None for p in children):
                raise RuntimeError(f'A ROS child process exited; inspect {args.output}/*.log')
            if self.plan is None:
                self.detect()
                if time.monotonic() - self.started > 30:
                    raise RuntimeError('No usable AprilTag detection within 30 seconds')
            elif self.index < len(self.plan):
                self.q, self.phase, attached = self.plan[self.index]
                if self.attached and not attached:
                    self.parked_tool = arm.fk(self.q) @ wrist_to_tool
                self.attached = attached
                self.index += 1
            elif not self.done:
                self.report['completed'] = True
                (args.output / 'report.json').write_text(json.dumps(self.report, indent=2) + '\n')
                self.done = True
                self.phase = 'COMPLETE'
                self.get_logger().info(f'Cycle complete; report: {args.output / "report.json"}')
            if self.phase != self.previous_phase:
                self.get_logger().info(self.phase)
                self.previous_phase = self.phase
            stamp = self.get_clock().now().to_msg()
            wrist = arm.fk(self.q)
            cam = wrist @ wrist_to_camera
            key = stamp.sec*10**9 + stamp.nanosec
            self.history[key] = cam
            while len(self.history) > 100:
                del self.history[next(iter(self.history))]
            self.broadcaster.sendTransform([
                self.tf('WAIST', 'wrist_camera_optical', cam, stamp),
                self.tf('WAIST', 'sim_fork_truth', fork_truth, stamp),
                self.tf('sim_fork_truth', 'sim_tag_mount', fork_to_tag, stamp),
                self.tf('WAIST', 'rhand_base_link',
                        wrist @ wrist_to_tool if self.attached else self.parked_tool, stamp)])
            js = JointState()
            js.header.stamp = stamp
            js.name = arm.all_names + ['lhand_joint', 'rhand_joint']
            positions = dict(zip(arm.names, self.q))
            positions.update(LARM_JOINT1=-.3, LARM_JOINT2=-1.4, LARM_JOINT4=.3)
            js.position = [float(positions.get(n, 0)) for n in js.name]
            self.joints.publish(js)
            pixels = camera.render(cam, tag_truth)
            if self.plan is None:
                cv2.imwrite(str(args.output / 'wrist_camera.png'), pixels)
            image = Image()
            image.header.frame_id, image.header.stamp = 'wrist_camera_optical', stamp
            image.height, image.width = pixels.shape
            image.encoding, image.step, image.data = 'mono8', image.width, pixels.tobytes()
            info = CameraInfo()
            info.header = image.header
            info.height, info.width = image.height, image.width
            info.distortion_model, info.d = 'plumb_bob', [0.]*5
            info.k, info.r = camera.k.flatten().tolist(), np.eye(3).flatten().tolist()
            info.p = np.column_stack([camera.k, np.zeros(3)]).flatten().tolist()
            self.info.publish(info)
            self.images.publish(image)
            self.draw(wrist, cam, stamp)

    rclpy.init(args=[])
    node = None
    try:
        with tempfile.TemporaryDirectory(prefix='fork_sim_') as tmp:
            params = Path(tmp) / 'params.yaml'
            params.write_text(yaml.safe_dump({
                'robot_state_publisher': {'ros__parameters': {'robot_description': urdf}},
                'tool_state_publisher': {'ros__parameters': {'robot_description': tool_urdf}},
                'apriltag': {'ros__parameters': {
                    'family': '36h11', 'size': config['tag_size'], 'max_hamming': 0,
                    'pose_estimation_method': 'pnp', 'detector.decimate': 1.0,
                    'tag.ids': [config['tag_id']], 'tag.frames': ['fork_tag'],
                    'tag.sizes': [config['tag_size']]}}}))
            spawn('robot_state_publisher', 'robot_state_publisher', ['--ros-args', '--params-file', str(params)])
            spawn('robot_state_publisher', 'robot_state_publisher', ['--ros-args',
                  '-r', '__node:=tool_state_publisher', '--params-file', str(params),
                  '-r', 'robot_description:=/tool/robot_description'])
            spawn('apriltag_ros', 'apriltag_node', ['--ros-args', '-r', '__node:=apriltag',
                  '--params-file', str(params), '-r', 'image_rect:=/wrist_camera/image_rect',
                  '-r', 'camera_info:=/wrist_camera/camera_info'])
            if not args.no_rviz:
                spawn('rviz2', 'rviz2', ['-d', str(share / 'rviz/apriltag_fork_sim.rviz')])
            node = Simulation()
            while rclpy.ok() and not (args.once and node.done):
                rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        for process in children:
            if process.poll() is None:
                process.terminate()
        for process in children:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in logs:
            log.close()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
