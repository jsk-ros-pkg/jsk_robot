"""YUBI description integration checks; no ROS master or hardware required."""
import struct
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml

from detachable_tools_common.description import (
    build_control_description, build_dynamixel_info, describe_hand, package_share_path,
)
from detachable_tools_common.tool_detection import select_tool


class YubiDescriptionTest(unittest.TestCase):
    def test_both_ros_naming_modes_and_detection(self):
        for side, motor_id, detacher_id in [('lhand', 8, 0), ('rhand', 9, 1)]:
            for prefix in (False, True):
                model = describe_hand(side, 'yubi', prefix=prefix)
                self.assertEqual(model.command_joint, f'{side}_yubi_joint' if prefix else f'{side}_joint')
                self.assertEqual({m.id for m in model.motors}, {motor_id, detacher_id})
                root = ET.fromstring(model.robot_description)
                links = {link.get('name') for link in root.findall('link')}
                self.assertIn(model.tip_link, links)
                for mesh in root.iter('mesh'):
                    self.assertEqual(mesh.get('scale'), '0.001 0.001 0.001')
                    self.assertTrue((package_share_path() / mesh.get('filename').split('package://detachable_tools/')[1]).is_file())
            self.assertEqual(select_tool(side, [detacher_id, motor_id]), 'yubi')
            self.assertEqual(select_tool(side, [motor_id], include_detacher=False), 'yubi')
            with self.assertRaises(RuntimeError):
                select_tool(side, [9 if side == 'lhand' else 8])
            info = build_dynamixel_info(side, 'yubi', include_detacher=False)
            self.assertEqual(info[f'{side}_motor']['ID'], motor_id)
            self.assertEqual(info[f'{side}_motor']['Operating_Mode'], 5)

    def test_combined_control_tree_and_controller(self):
        for side in ('lhand', 'rhand'):
            root = ET.fromstring(build_control_description(side))
            for kind in ('link', 'joint', 'ros2_control'):
                names = [e.get('name') for e in root.findall(kind)]
                self.assertEqual(len(names), len(set(names)))
            links = {e.get('name') for e in root.findall('link')}
            children = [j.find('child').get('link') for j in root.findall('joint')]
            self.assertEqual(len(children), len(set(children)))
            self.assertEqual(links - set(children), {f'{side}_base_link'})
            config = yaml.safe_load((package_share_path() / f'config/{side}/controllers.yaml').read_text())
            self.assertIn(f'{side}_yubi', config['/**/controller_manager']['ros__parameters']['hardware_components_initial_state']['unconfigured'])
            self.assertEqual(config['/**/yubi_gripper_controller']['ros__parameters']['joint'], f'{side}_yubi_joint')

    def test_binary_stl_is_complete(self):
        data = (package_share_path() / 'meshes/yubi/yubi_xc330_front_flat_with_interface.stl').read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        self.assertGreater(count, 0)
        self.assertEqual(len(data), 84 + 50 * count)


if __name__ == '__main__':
    unittest.main()
