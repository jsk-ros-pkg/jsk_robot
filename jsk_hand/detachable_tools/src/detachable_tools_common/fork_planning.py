"""Observation and exchange path planning for the Nextage fork API (SI units)."""
import numpy as np
from scipy.spatial.transform import Rotation

from .fork_simulation import transform, pose_error


def make_plan(arm, q, fork, config, operation="cycle"):
    """Precompute and validate every Cartesian sample before starting playback."""
    dt = 1.0 / config['rate']
    dock = fork @ transform(config['fork_to_wrist_xyz'], config['fork_to_wrist_rpy'])
    approach = dock.copy()
    approach[:3, 3] += fork[:3, :3] @ np.array(config['approach_offset'])
    above = dock.copy()
    above[:3, 3] += fork[:3, :3] @ np.array([0, 0, config['lift_distance']])
    observe = observation_pose(fork, np.array(config['wrist_to_camera']), config)
    observe_attach = observation_pose(fork, np.array(config['wrist_to_camera']), config, stage='attach')
    visibility = {'VERIFY_DETACH': observation_visibility(observe, fork, config),
                  'VERIFY_ATTACH': observation_visibility(observe_attach, fork, config)}
    for name, view, anchor in [('VERIFY_DETACH', observe, approach),
                               ('VERIFY_ATTACH', observe_attach, above)]:
        translation, rotation = pose_error(view, anchor)
        visibility[name].update(wrist_offset_from_staging_m=float(translation),
                                wrist_rotation_from_staging_deg=float(np.rad2deg(rotation)))
    # The original entrance-side tag is invisible at horizontal docking poses.
    # Stop at a reachable, tag-facing pose before each insertion operation.
    detach_phases = [('OBSERVE_DETACH', observe, True), ('VERIFY_DETACH', observe, True),
                     ('APPROACH', approach, True), ('INSERT_SIDEWAYS', dock, True),
                     ('DETACH', dock, False), ('LIFT_UP', above, False)]
    attach_phases = [('OBSERVE_ATTACH', observe_attach, False), ('VERIFY_ATTACH', observe_attach, False),
                     ('APPROACH', above, False), ('LOWER_TO_ATTACH', dock, False),
                     ('ATTACH', dock, True), ('WITHDRAW_SIDEWAYS', approach, True)]
    if operation == 'detach':
        phases = detach_phases
    elif operation == 'attach':
        phases = attach_phases
    elif operation == 'cycle':
        phases = detach_phases + attach_phases
    else:
        raise ValueError('operation must be detach, attach, or cycle')
    frames = []
    max_pos = max_rot = 0.
    for name, target, attached in phases:
        start = arm.fk(q)
        # Orientation is constant except for the small perception correction on approach.
        from scipy.spatial.transform import Slerp
        rotations = Slerp([0, 1], Rotation.from_matrix([start[:3, :3], target[:3, :3]]))
        seconds = config.get('approach_seconds', config['segment_seconds']) if name == 'APPROACH' or name.startswith('OBSERVE_') else config['segment_seconds']
        if name.startswith('OBSERVE_'):
            seconds = config.get('observation_seconds', seconds)
        count = max(2, round(seconds / dt))
        if name.startswith('VERIFY_'):
            count = 1
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
    return frames, {'samples': len(frames),
                    'observation_plan': [dict(phase=name, **visibility[name])
                                         for name, _, _ in phases if name.startswith('VERIFY_')], 'max_ik_position_error_m': max_pos,
                    'max_ik_rotation_error_rad': max_rot,
                    'max_joint_velocity_rad_s': float(velocities.max())}


def observation_pose(fork, wrist_camera, config, stage="detach"):
    """Aim at the fixed tag with stage-specific position and optical-axis roll."""
    key = 'observation_attach_camera_xyz' if stage == 'attach' else 'observation_camera_xyz'
    position = np.array(config.get(key, config['observation_camera_xyz']), dtype=float)
    forward = np.array(config['fork_to_tag_xyz']) - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    camera = transform(position)
    roll_key = 'observation_attach_camera_roll' if stage == 'attach' else 'observation_camera_roll'
    camera[:3, :3] = np.column_stack([right, down, forward]) @ Rotation.from_euler(
        'z', config.get(roll_key, 0.)).as_matrix()
    return fork @ camera @ np.linalg.inv(wrist_camera)


def observation_visibility(wrist, fork, config):
    """Check the complete tag and quiet border in the planned pinhole image."""
    camera = wrist @ np.array(config['wrist_to_camera'])
    tag = fork @ transform(config['fork_to_tag_xyz'], config['fork_to_tag_rpy'])
    relative = np.linalg.inv(camera) @ tag
    s = config['tag_size'] * .625
    corners = np.array([[-s,s,0,1], [s,s,0,1], [s,-s,0,1], [-s,-s,0,1]])
    vertices = (relative @ corners.T).T[:, :3]
    width, height = config['image_size']
    pixels = config['focal_length'] * vertices[:, :2] / vertices[:, 2:]
    pixels += [width/2, height/2]
    margin = float(min(pixels[:,0].min(), width-pixels[:,0].max(),
                       pixels[:,1].min(), height-pixels[:,1].max()))
    if np.any(vertices[:,2] <= .01) or relative[2,2] >= 0 or margin < 10:
        raise RuntimeError('Planned observation does not contain the tag with a 10 pixel margin')
    return {'camera_xyz_in_fork_m': (np.linalg.inv(fork) @ camera)[:3,3].tolist(),
            'quiet_border_margin_px': margin,
            'tag_distance_m': float(np.linalg.norm(relative[:3,3])),
            'scope': 'pinhole visibility; no scene occlusion check'}
