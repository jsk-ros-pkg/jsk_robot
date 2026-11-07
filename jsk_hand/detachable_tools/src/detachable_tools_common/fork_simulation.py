"""Kinematics and ideal pinhole camera for the ROS-only fork demo (SI units)."""
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    t[:3, 3] = xyz
    return t


def pose_error(actual, target):
    return (np.linalg.norm(actual[:3, 3] - target[:3, 3]),
            Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).magnitude())


class RightArm:
    """Nextage right-arm IK with the chest fixed at zero; no hardware interface."""
    def __init__(self, urdf):
        root = ET.fromstring(urdf)
        self.names = [f'RARM_JOINT{i}' for i in range(6)]
        joints = [root.find(f"joint[@name='{name}']") for name in self.names]
        self.origins = [transform(np.fromstring(j.find('origin').get('xyz'), sep=' '),
                                  np.fromstring(j.find('origin').get('rpy'), sep=' '))
                        for j in joints]
        self.axes = [np.fromstring(j.find('axis').get('xyz'), sep=' ') for j in joints]
        self.lower = np.array([float(j.find('limit').get('lower')) for j in joints])
        self.upper = np.array([float(j.find('limit').get('upper')) for j in joints])
        self.velocity = np.array([float(j.find('limit').get('velocity')) for j in joints])
        self.all_names = [j.get('name') for j in root.findall('joint')
                          if j.get('type') == 'revolute']

    def fk(self, q):
        t = np.eye(4)
        for origin, axis, value in zip(self.origins, self.axes, q):
            r = np.eye(4)
            r[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
            t = t @ origin @ r
        return t

    def ik(self, target, seed):
        def residual(q):
            t = self.fk(q)
            return np.r_[t[:3, 3] - target[:3, 3], .2 * Rotation.from_matrix(
                target[:3, :3].T @ t[:3, :3]).as_rotvec()]
        result = least_squares(residual, seed, bounds=(self.lower, self.upper),
                               max_nfev=250, ftol=1e-11, xtol=1e-11, gtol=1e-11)
        pos, rot = pose_error(self.fk(result.x), target)
        if pos > .0001 or rot > .001:
            raise RuntimeError(f'IK failed: {pos:.6f} m, {rot:.6f} rad')
        return result.x


def make_plan(arm, q, fork, config):
    """Precompute and validate every Cartesian sample before starting playback."""
    dt = 1.0 / config['rate']
    dock = fork @ transform(config['fork_to_wrist_xyz'], config['fork_to_wrist_rpy'])
    approach = dock.copy()
    approach[:3, 3] += fork[:3, :3] @ np.array(config['approach_offset'])
    above = dock.copy()
    above[:3, 3] += fork[:3, :3] @ np.array([0, 0, config['lift_distance']])
    phases = [('APPROACH', approach, True), ('INSERT_SIDEWAYS', dock, True),
              ('DETACH', dock, False), ('LIFT_UP', above, False),
              ('LOWER_TO_ATTACH', dock, False), ('ATTACH', dock, True),
              ('WITHDRAW_SIDEWAYS', approach, True)]
    frames = []
    max_pos = max_rot = 0.
    for name, target, attached in phases:
        start = arm.fk(q)
        # Orientation is constant except for the small perception correction on approach.
        from scipy.spatial.transform import Slerp
        rotations = Slerp([0, 1], Rotation.from_matrix([start[:3, :3], target[:3, :3]]))
        count = max(2, round(config['segment_seconds'] / dt))
        if name in ('DETACH', 'ATTACH'):
            count = round(config['latch_seconds'] / dt)
        segment = []
        for i in range(1, count + 1):
            u = i / count
            s = u*u*(3 - 2*u)
            desired = np.eye(4)
            desired[:3, 3] = start[:3, 3]*(1-s) + target[:3, 3]*s
            desired[:3, :3] = rotations(s).as_matrix()
            next_q = arm.ik(desired, q)
            if np.max(np.abs(next_q - q)) > .15:
                raise RuntimeError('Discontinuous IK branch')
            pe, re = pose_error(arm.fk(next_q), desired)
            max_pos, max_rot = max(max_pos, pe), max(max_rot, re)
            segment.append((next_q.copy(), name, attached))
            q = next_q
        frames.extend(segment)
    velocities = np.abs(np.diff(np.array([f[0] for f in frames]), axis=0)) / dt
    if np.any(velocities > arm.velocity + 1e-6):
        raise RuntimeError('URDF velocity limit exceeded; increase segment_seconds')
    return frames, {'samples': len(frames), 'max_ik_position_error_m': max_pos,
                    'max_ik_rotation_error_rad': max_rot,
                    'max_joint_velocity_rad_s': float(velocities.max())}


class TagCamera:
    def __init__(self, config):
        self.width, self.height = config['image_size']
        self.k = np.array([[config['focal_length'], 0, self.width/2],
                           [0, config['focal_length'], self.height/2], [0, 0, 1.]])
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, 'generateImageMarker'):
            self.bitmap = cv2.aruco.generateImageMarker(dictionary, config['tag_id'], 240)
        else:
            self.bitmap = cv2.aruco.drawMarker(dictionary, config['tag_id'], 240)
        # OpenCV's dictionary bitmap is rotated 180 degrees relative to the
        # AprilRobotics canonical image (and apriltag_ros PnP corner ordering).
        self.bitmap = cv2.rotate(self.bitmap, cv2.ROTATE_180)
        self.size = config['tag_size']

    def render(self, camera, tag):
        # Tag +x right, +y up, +z out of printed face. Optical camera +z forward.
        s = self.size/2
        corners = np.array([[-s,s,0,1], [s,s,0,1], [s,-s,0,1], [-s,-s,0,1]])
        relative = np.linalg.inv(camera) @ tag
        vertices = (relative @ corners.T).T[:, :3]
        blank = np.full((self.height, self.width), 230, np.uint8)
        if np.any(vertices[:, 2] <= .01) or relative[2, 2] >= 0:
            return blank
        pixels = (self.k @ vertices.T).T
        pixels = (pixels[:, :2] / pixels[:, 2:]).astype(np.float32)
        # Include a white quiet zone around the black tag border.
        patch = cv2.copyMakeBorder(self.bitmap, 30, 30, 30, 30, cv2.BORDER_CONSTANT, value=255)
        source = np.float32([[30,30], [270,30], [270,270], [30,270]])
        homography = cv2.getPerspectiveTransform(source, pixels)
        return cv2.warpPerspective(patch, homography, (self.width, self.height),
                                   flags=cv2.INTER_LINEAR, borderValue=230)
