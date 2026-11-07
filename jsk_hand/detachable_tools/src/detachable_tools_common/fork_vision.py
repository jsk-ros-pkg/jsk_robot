"""Fresh-image confirmation shared by simulation and the hardware API."""
import numpy as np
from scipy.spatial.transform import Rotation
from .fork_simulation import pose_error


class VisionGate:
    def __init__(self, start_ns, reference, config):
        self.start_ns = start_ns
        self.reference = reference
        self.config = config
        self.samples = {}
        self.after_ns = start_ns + round(config.get('vision_settle_seconds', .2)*1e9)

    def add(self, stamp_ns, pose, now_ns):
        if now_ns-self.start_ns > self.config.get('vision_timeout_seconds', 3.)*1e9:
            raise RuntimeError('TAG_REACQUISITION_TIMEOUT: no stable fresh detection; motion stopped')
        if stamp_ns is None or pose is None or stamp_ns <= self.after_ns:
            return None
        age = (now_ns-stamp_ns)/1e9
        if age < 0 or age > self.config.get('max_tag_age', .5):
            return None
        self.samples[stamp_ns] = pose.copy()
        if len(self.samples) < self.config.get('vision_confirm_frames', 5):
            return None
        poses = list(self.samples.values())
        estimate = np.eye(4)
        estimate[:3,3] = np.mean([p[:3,3] for p in poses],axis=0)
        estimate[:3,:3] = Rotation.from_matrix([p[:3,:3] for p in poses]).mean().as_matrix()
        pos_limit = self.config.get('vision_position_tolerance_m', .001)
        rot_limit = np.deg2rad(self.config.get('vision_rotation_tolerance_deg', 2.))
        for p in poses + [self.reference]:
            pe,re = pose_error(p,estimate)
            if pe > pos_limit or re > rot_limit:
                raise RuntimeError(f'TAG_POSE_CHANGED: {pe*1000:.3f} mm / {np.rad2deg(re):.3f} deg; '
                                   'replan required before further motion')
        pe,re = pose_error(self.reference,estimate)
        return {'fresh_frames':len(poses), 'translation_change_m':float(pe),
                'rotation_change_deg':float(np.rad2deg(re)),
                'first_image_stamp_ns':min(self.samples), 'checkpoint_start_ns':self.start_ns}
