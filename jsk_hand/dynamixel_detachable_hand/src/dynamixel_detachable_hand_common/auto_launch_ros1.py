"""Discover both buses before starting ROS 1 ros_control hardware nodes."""
from pathlib import Path
from xml.etree import ElementTree as ET

from .tool_detection import detacher_available, scan_ids, select_tool


def launch_xml(package_path, detections, include_detacher, baud_rate, hand_effort_limit):
    root = ET.Element('launch')
    for side, port, ids, tool in detections:
        include = ET.SubElement(root, 'include', file=str(Path(package_path) / 'launch/hand_control.launch'))
        args = dict(side=side, namespace=side, port_name=port, baud_rate=str(baud_rate),
                    tool=tool, attached=str(tool != 'detacher').lower(),
                    include_detacher=str(include_detacher and detacher_available(side, ids)).lower(),
                    detected_ids=','.join(str(i) for i in sorted(ids)),
                    hand_effort_limit=str(hand_effort_limit))
        for name, value in args.items():
            ET.SubElement(include, 'arg', name=name, value=value)
    return ET.tostring(root, encoding='unicode')


def main():
    import roslaunch
    import rospkg
    import rospy

    rospy.init_node('dual_hand_auto')
    include_detacher = rospy.get_param('~include_detacher', True)
    baud_rate = rospy.get_param('~baud_rate', 57600)
    detections = []
    for side, key in [('lhand', 'left_port_name'), ('rhand', 'right_port_name')]:
        port = rospy.get_param(f'~{key}', f'/dev/{side}')
        ids = scan_ids(port, baud_rate)
        tool = select_tool(side, ids, include_detacher)
        if include_detacher and not detacher_available(side, ids):
            rospy.logwarn("%s: detacher did not respond; tool changing disabled, using %s only", side, tool)
        rospy.loginfo('%s (%s arm): port=%s, detected IDs=%s, tool=%s, attached=%s',
                      side, 'left' if side == 'lhand' else 'right', port,
                      sorted(ids), tool, tool != 'detacher')
        detections.append((side, port, ids, tool))
    xml = launch_xml(rospkg.RosPack().get_path('dynamixel_detachable_hand'), detections,
                     include_detacher, baud_rate, rospy.get_param('~hand_effort_limit', 0.92))
    parent = roslaunch.parent.ROSLaunchParent(rospy.get_param('/run_id'), [], roslaunch_strs=[xml])
    try:
        parent.start()
        parent.spin()
    finally:
        parent.shutdown()


if __name__ == '__main__':
    main()
