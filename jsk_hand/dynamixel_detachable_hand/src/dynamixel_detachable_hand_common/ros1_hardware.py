"""ROS 1 tool switching: update the parameters and restart only dynamixel_general_control."""

from __future__ import annotations

import time

import rospy

from .description import HandModel, build_dynamixel_info

HARDWARE_NODE = "dynamixel_general_control"
ROBOT_STATE_PUBLISHER = "robot_state_publisher"
JOINT_STATE_CONTROLLER = "joint_state_controller"
TRAJECTORY_CONTROLLER = "position_joint_trajectory_controller"
EFFORT_CONTROLLER = "joint_group_effort_controller"


def _controllers(model: HandModel, include_detacher: bool) -> tuple[list[str], list[str]]:
    """Controllers to start and the joint_group_effort_controller joints."""
    effort_joints = [model.command_joint] if model.attached else []
    if include_detacher or not model.attached:
        effort_joints.append(model.detach_joint)
    controllers = [JOINT_STATE_CONTROLLER, EFFORT_CONTROLLER]
    if model.attached:
        controllers.append(TRAJECTORY_CONTROLLER)
    return controllers, effort_joints


def set_controller_params(model: HandModel, include_detacher: bool) -> None:
    _, effort_joints = _controllers(model, include_detacher)
    rospy.set_param(f"{EFFORT_CONTROLLER}/type", "effort_controllers/JointGroupEffortController")
    rospy.set_param(f"{EFFORT_CONTROLLER}/joints", effort_joints)
    if model.attached:
        rospy.set_param(f"{TRAJECTORY_CONTROLLER}/type", "position_controllers/JointTrajectoryController")
        rospy.set_param(f"{TRAJECTORY_CONTROLLER}/allow_partial_joints_goal", True)
        rospy.set_param(f"{TRAJECTORY_CONTROLLER}/joints", [model.command_joint])


def start_controllers(model: HandModel, include_detacher: bool, hand_effort_limit: float, timeout=10.0) -> bool:
    from controller_manager_msgs.srv import ListControllers, LoadController, SwitchController
    from std_msgs.msg import Float64MultiArray

    controllers, effort_joints = _controllers(model, include_detacher)
    try:
        rospy.wait_for_service("controller_manager/switch_controller", timeout)
    except rospy.ROSException:
        rospy.logerr("controller_manager is not available")
        return False
    loaded = {c.name for c in rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)().controller}
    load = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
    for name in controllers:
        if name not in loaded and not load(name).ok:
            rospy.logerr(f"failed to load {name}")
            return False
    switch = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)
    if not switch(start_controllers=controllers, stop_controllers=[], strictness=1).ok:
        return False
    # Goal Current limit of the hand motor; position_controllers ignore the trajectory effort
    pub = rospy.Publisher(f"{EFFORT_CONTROLLER}/command", Float64MultiArray, queue_size=1, latch=True)
    msg = Float64MultiArray(data=[hand_effort_limit if j == model.command_joint else 0.0 for j in effort_joints])
    pub.publish(msg)
    return True


def _node_uri(name: str) -> str | None:
    import rosgraph

    try:
        return rosgraph.Master(rospy.get_name()).lookupNode(rospy.resolve_name(name))
    except Exception:
        return None


def restart_hardware(model: HandModel, include_detacher: bool, hand_effort_limit: float, timeout=10.0) -> bool:
    import rosnode

    info = build_dynamixel_info(
        model.side, model.tool, attached=model.attached, include_detacher=include_detacher
    )
    rospy.set_param(f"{HARDWARE_NODE}/dynamixel_info", info)
    rospy.set_param("robot_description", model.robot_description)
    set_controller_params(model, include_detacher)
    nodes = [rospy.resolve_name(HARDWARE_NODE), rospy.resolve_name(ROBOT_STATE_PUBLISHER)]
    old_uri = _node_uri(HARDWARE_NODE)
    rosnode.kill_nodes(nodes)
    deadline = time.time() + timeout
    while time.time() < deadline:
        uri = _node_uri(HARDWARE_NODE)
        if uri is not None and uri != old_uri:
            break
        time.sleep(0.1)
    else:
        rospy.logerr(f"{HARDWARE_NODE} was not respawned (respawn=\"true\" in the launch file?)")
        return False
    return start_controllers(model, include_detacher, hand_effort_limit, max(1.0, deadline - time.time()))
