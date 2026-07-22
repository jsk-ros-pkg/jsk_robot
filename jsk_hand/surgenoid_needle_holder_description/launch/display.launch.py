import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("surgenoid_needle_holder_description")
    urdf = os.path.join(pkg, "urdf", "surgenoid_needle_holder_description.urdf")
    with open(urdf) as f:
        robot_description = f.read()
    rviz = os.path.join(pkg, "rviz", "surgenoid_needle_holder_description.rviz")
    closures = os.path.join(pkg, "config", "loop_closures.yaml")
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            remappings=[("joint_states", "joint_states_source")],
        ),
        Node(
            package="surgenoid_needle_holder_description",
            executable="loop_closure_relay.py",
            output="screen",
            parameters=[{"urdf_file": urdf, "config_file": closures}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz],
        ),
    ])
