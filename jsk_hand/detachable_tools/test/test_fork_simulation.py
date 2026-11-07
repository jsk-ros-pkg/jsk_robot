"""Offline regression checks; end-to-end detector validation is --once --no-rviz."""
from pathlib import Path
import unittest

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from detachable_tools_common.fork_simulation import RightArm, TagCamera, make_plan, transform, split_jsk_model, fixed_transform


class ForkSimulationTest(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[1]
        self.config = yaml.safe_load((root / 'config/fork_simulation.yaml').read_text())
        urdf = Path(get_package_share_directory('nextage_tutorials')) / 'urdf/JSKNextageOpenGripper.urdf'
        self.robot_xml, self.tool_xml, self.wrist_tool, self.wrist_camera = split_jsk_model(urdf.read_text(), root)
        self.arm = RightArm(self.robot_xml)
        mount = transform(self.config['fork_to_tool_xyz'], self.config['fork_to_tool_rpy']) @ np.linalg.inv(self.wrist_tool)
        from scipy.spatial.transform import Rotation
        self.config['fork_to_wrist_xyz'] = mount[:3,3]
        self.config['fork_to_wrist_rpy'] = Rotation.from_matrix(mount[:3,:3]).as_euler('xyz')
        self.fork = transform(self.config['fork_xyz'], self.config['fork_rpy'])
        self.dock = self.fork @ transform(self.config['fork_to_wrist_xyz'], self.config['fork_to_wrist_rpy'])
        approach = self.dock.copy()
        approach[:3, 3] += self.config['approach_offset']
        self.q = self.arm.ik(approach, [-.28,-.20,-1.86,.27,.40,1.19])

    def test_complete_path_and_latch_order(self):
        frames, report = make_plan(self.arm, self.q, self.fork, self.config)
        phases = list(dict.fromkeys(name for _, name, _ in frames))
        self.assertEqual(phases, ['APPROACH', 'INSERT_SIDEWAYS', 'DETACH', 'LIFT_UP',
                                  'LOWER_TO_ATTACH', 'ATTACH', 'WITHDRAW_SIDEWAYS'])
        for q, phase, attached in frames:
            self.assertTrue(np.all(q >= self.arm.lower) and np.all(q <= self.arm.upper))
            self.assertEqual(attached, phase not in ('DETACH', 'LIFT_UP', 'LOWER_TO_ATTACH'))
        raised = [q for q, phase, _ in frames if phase == 'LIFT_UP'][-1]
        np.testing.assert_allclose(self.arm.fk(raised)[:3,3], self.dock[:3,3]+[0,0,.12], atol=1e-4)
        np.testing.assert_allclose(frames[-1][0], self.q, atol=1e-5)
        self.assertLess(report['max_ik_position_error_m'], 1e-4)

    def test_jsk_tool_separation(self):
        import xml.etree.ElementTree as ET
        robot, tool = ET.fromstring(self.robot_xml), ET.fromstring(self.tool_xml)
        self.assertIsNotNone(robot.find("link[@name='RARM_NEJINEJI_Link']"))
        self.assertIsNone(robot.find("link[@name='rhand_base_link']"))
        self.assertIsNotNone(tool.find("link[@name='rhand_screwlock_male_1']"))
        self.assertIsNone(robot.find("joint[@name='RARM_NEJINEJI_TO_GRIPPER']"))
        # Reattachment exactly recovers the source URDF mounting transform.
        fork_tool = self.fork @ transform(self.config['fork_to_tool_xyz'], self.config['fork_to_tool_rpy'])
        np.testing.assert_allclose(self.dock @ self.wrist_tool, fork_tool, atol=1e-8)

    def test_unreachable_pose_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'IK failed'):
            self.arm.ik(transform([10,0,0]), self.q)

    def test_canonical_tag_and_out_of_view(self):
        camera = TagCamera(self.config)
        bits = cv2.resize(camera.bitmap, (8,8), interpolation=cv2.INTER_NEAREST)
        # AprilRobotics/tag36h11/tag36_11_00000.png: first inner row.
        np.testing.assert_array_equal(bits[1,1:7] > 0, [True,True,False,True,False,True])
        cam = self.arm.fk(self.q) @ self.wrist_camera
        tag = self.fork @ transform(self.config['fork_to_tag_xyz'], self.config['fork_to_tag_rpy'])
        self.assertGreater(np.count_nonzero(camera.render(cam, tag) == 0), 100)
        behind = cam @ transform([0,0,-1])
        self.assertTrue(np.all(camera.render(cam, behind) == 230))


if __name__ == '__main__':
    unittest.main()
