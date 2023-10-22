#!/usr/bin/env python

import actionlib
import imp
import rospkg
import rospy
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    MotionPlanResponse,
    PlanningScene,
    PlanningSceneWorld,
    RobotState,
    MoveGroupAction,
    MoveGroupResult,
)  # importing noetic version
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetMotionPlan,
    GetPlanningScene,
    GetStateValidity,
    GetPositionIK,
)  # importing noetic version
from packaging import version


"""
On https://github.com/ros-planning/moveit_msgs/compare/0.10.1...0.11.4, the CollisionObject.msg has breaking changes
It affects the msgs, AttachedCollisionObject.msg, PlanningSceneWorld.msg and RobotState.msg.

In fetch, these moveit_msgs's topics are advertised
```
obinata@fetch1075:~ $ rostopic list -v | grep moveit_msgs
 * /place/result [moveit_msgs/PlaceActionResult] 1 publisher
 * /move_group/monitored_planning_scene [moveit_msgs/PlanningScene] 1 publisher
 * /pickup/feedback [moveit_msgs/PickupActionFeedback] 1 publisher
 * /move_group/feedback [moveit_msgs/MoveGroupActionFeedback] 1 publisher
 * /execute_trajectory/feedback [moveit_msgs/ExecuteTrajectoryActionFeedback] 1 publisher
 * /place/feedback [moveit_msgs/PlaceActionFeedback] 1 publisher
 * /pickup/result [moveit_msgs/PickupActionResult] 1 publisher
 * /move_group/display_planned_path [moveit_msgs/DisplayTrajectory] 1 publisher
 * /planning_scene_world [moveit_msgs/PlanningSceneWorld] 1 publisher
 * /move_group/result [moveit_msgs/MoveGroupActionResult] 1 publisher
 * /execute_trajectory/result [moveit_msgs/ExecuteTrajectoryActionResult] 1 publisher
 * /collision_object [moveit_msgs/CollisionObject] 1 subscriber
 * /planning_scene_world [moveit_msgs/PlanningSceneWorld] 1 subscriber
 * /move_group/goal [moveit_msgs/MoveGroupActionGoal] 1 subscriber
 * /place/goal [moveit_msgs/PlaceActionGoal] 1 subscriber
 * /attached_collision_object [moveit_msgs/AttachedCollisionObject] 1 subscriber
 * /pickup/goal [moveit_msgs/PickupActionGoal] 1 subscriber
 * /execute_trajectory/goal [moveit_msgs/ExecuteTrajectoryActionGoal] 1 subscriber
 * /planning_scene [moveit_msgs/PlanningScene] 1 subscriber
```
and the topic depends on CollisionObjects are
```
/place moveit_msgs/PlaceAction
/move_group/monitored_planning_scene moveit_msgs/PlanningScene
/pickup moveit_msgs/PickupAction
/move_group moveit_msgs/MoveGroupAction
/move_group/display_planned_path moveit_msgs/DisplayTrajectory
/planning_scene_world moveit_msgs/PlanningSceneWorld
/collision_object moveit_msgs/CollisionObject
/attached_collision_object moveit_msgs/AttachedCollisionObject
/planning_scene moveit_msgs/PlanningSceneWorld
```

these moveit_msgs's services may be advertised
```
obinata@fetch1075:/opt/ros/melodic/share/moveit_core $ rosnode info /move_group -q
...
Services
 * /apply_planning_scene
 * /check_state_validity
 * /clear_octomap
 * /compute_cartesian_path
 * /compute_fk
 * /compute_ik
 * /get_planner_params
 * /get_planning_scene
 * /move_group/get_loggers
 * /move_group/load_map
 * /move_group/ompl/set_parameters
 * /move_group/plan_execution/set_parameters
 * /move_group/planning_scene_monitor/set_parameters
 * /move_group/save_map
 * /move_group/sense_for_plan/set_parameters
 * /move_group/set_logger_level
 * /move_group/trajectory_execution/set_parameters
 * /plan_kinematic_path
 * /query_planner_interface
 * /set_planner_params

```
and the services depend on CollisionObjects are
```
/apply_planning_scene moveit_msgs/ApplyPlanningScene
/check_state_validity moveit_msgs/GetStateValidity
/compute_cartesian_path moveit_msgs/GetCartesianPath
/compute_fk moveit_msgs/GetPositionFK
/compute_ik moveit_msgs/GetPositionIK
/get_planning_scene moveit_msgs/GetPlanningScene
/plan_kinematic_path moveit_msgs/GetMotionPlan
```
"""


class MoveitNoeticBridge(object):
    """
    This node is expected to work on melodic PC
    """

    def __init__(self):
        # Loading Melodic version services, c.f. https://stackoverflow.com/questions/67631/how-can-i-import-a-module-dynamically-given-the-full-path
        self._melodic_apply_planning_scene = imp.load_source(
            "ApplyPlanningScene",
            "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_msgs/srv/_ApplyPlanningScene.py",
        )
        self._melodic_get_motion_plan = imp.load_source(
            "GetMotionPlan",
            "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_msgs/srv/_GetMotionPlan.py",
        )
        self._melodic_get_state_validity = imp.load_source(
            "GetStateValidity",
            "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_msgs/srv/_GetStateValidity.py",
        )
        self._melodic_get_position_ik = imp.load_source(
            "GetPositionIK",
            "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_msgs/srv/_GetPositionIK.py",
        )
        self._melodic_get_planning_scene = imp.load_source(
            "GetPlanningScene",
            "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_msgs/srv/_GetPlanningScene.py",
        )

        # Loading Melodic version messages
        self._melodic_collision_object = imp.load_source(
            "CollisionObject",
            "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_msgs/msg/_CollisionObject.py",
        )
        self._melodic_attached_collision_object = imp.load_source(
            "AttachedCollisionObject",
            "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_msgs/msg/_AttachedCollisionObject.py",
        )
        self._melodic_motion_plan_request = imp.load_source(
            "MotionPlanRequest",
            "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_msgs/msg/_MotionPlanRequest.py",
        )
        self._melodic_position_ik_request = imp.load_source(
            "PositionIKRequest",
            "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_msgs/msg/_PositionIKRequest.py",
        )
        self._melodic_planning_scene = imp.load_source(
            "PlanningScene",
            "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_msgs/msg/_PlanningScene.py",
        )
        self._melodic_planning_scene_world = imp.load_source(
            "PlanningSceneWorld",
            "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_msgs/msg/_PlanningSceneWorld.py",
        )
        self._melodic_robot_state = imp.load_source(
            "RobotState",
            "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_msgs/msg/_RobotState.py",
        )
        self._melodic_move_group_action = imp.load_source(
            "MoveGroupAction",
            "/opt/ros/melodic/lib/python2.7/dist-package/moveit_msgs/msg/_MoveGroupAction.py",
        )
        self._melodic_move_group_goal = imp.load_source(
            "MoveGroupGoal",
            "/opt/ros/melodic/lib/python2.7/dist-package/moveit_msgs/msg/_MoveGroupGoal.py",
        )
        self._melodic_move_group_status = imp.load_source(
            "MoveGroupStatus",
            "/opt/ros/melodic/lib/python2.7/dist-package/moveit_msgs/msg/_MoveGroupStatus.py",
        )
        self._melodic_move_group_result = imp.load_source(
            "MoveGroupResult",
            "/opt/ros/melodic/lib/python2.7/dist-package/moveit_msgs/msg/_MoveGroupResult.py",
        )

        # Service bridge
        self.apply_planning_scene_srv = rospy.Service(
            "/apply_planning_scene_noetic",
            ApplyPlanningScene,
            self._apply_planning_scene_srv_cb,
        )
        self.check_state_validity_srv = rospy.Service(
            "/check_state_validity_noetic",
            GetStateValidity,
            self._check_state_validity_srv_cb,
        )
        self.compute_ik_srv = rospy.Service(
            "/compute_ik_noetic", GetPositionIK, self._compute_ik_srv_cb
        )
        self.get_planning_scene_srv = rospy.Service(
            "/get_planning_scene_noetic",
            GetPlanningScene,
            self._get_planning_scene_srv_cb,
        )
        self.plan_kinematic_path_srv = rospy.Service(
            "/plan_kinematic_path_noetic",
            GetMotionPlan,
            self._plan_kinematic_path_srv_cb,
        )  # NOTE Is this service necessary to be bridged? Not seen in pr2eus_moveit

        # Service proxy
        self.apply_planning_scene_proxy = rospy.ServiceProxy(
            "/apply_planning_scene",
            self._melodic_apply_planning_scene.ApplyPlanningScene,
        )
        self.check_state_validity_proxy = rospy.ServiceProxy(
            "/check_state_validity", self._melodic_get_state_validity.GetStateValidity
        )
        self.compute_ik_proxy = rospy.ServiceProxy(
            "/compute_ik", self._melodic_get_position_ik.GetPositionIK
        )
        self.get_planning_scene_proxy = rospy.ServiceProxy(
            "/get_planning_scene", self._melodic_get_planning_scene.GetPlanningScene
        )
        self.plan_kinematic_path_proxy = rospy.ServiceProxy(
            "/plan_kinematic_path", self._melodic_get_motion_plan.GetMotionPlan
        )

        # Actionlib
        self.move_group_as = actionlib.SimpleActionServer(
            "/move_group_noetic",
            MoveGroupAction,
            execute_cb=self._move_group_action_cb,
            auto_start=True,
        )
        self.move_group_ac = actionlib.SimpleActionClient(
            "/move_group", self._melodic_move_group_action.MoveGroupAction
        )

        # Topic
        self.planning_scene_world_cb = rospy.Subscriber(
            "/planning_scene_world_noetic",
            PlanningSceneWorld,
            self._planning_scene_world_cb,
        )

        self.planning_scene_world_pub = rospy.Publisher(
            "/planning_scene_world",
            self._melodic_planning_scene_world.PlanningSceneWorld,
            queue_size=1,
        )

    def _apply_planning_scene_srv_cb(self, request):
        bridged_request = self._melodic_apply_planning_scene.ApplyPlanningSceneRequest()
        bridged_request.scene = self._convert_noetic_planning_scene_msg_to_melodic(
            request.scene
        )
        original_response = self.get_planning_scene_proxy.call(bridged_request)
        response = self._melodic_apply_planning_scene.ApplyPlanningSceneResponse()
        response.success = original_response.success
        return response

    def _check_state_validity_srv_cb(self, request):
        bridged_request = self._melodic_get_state_validity.GetStateValidityRequest()
        bridged_request.robot_state = self._convert_noetic_robot_state_msg_to_melodic(
            request.robot_state
        )
        bridged_request.group_name = request.group_name
        bridged_request.constraints = request.constraints
        original_response = self.check_state_validity_proxy.call(bridged_request)
        response = self._melodic_get_state_validity.GetStateValidityResponse()
        response.valid = original_response.valid
        response.contacts = original_response.contacts
        response.cost_sources = original_response.cost_sources
        response.constraints = original_response.constraints
        return response

    def _compute_ik_srv_cb(self, request):
        bridged_request = self._melodic_get_position_ik.GetPositionIKRequest()
        bridged_request.ik_request = (
            self._convert_noetic_position_ik_request_msg_to_melodic(request.ik_request)
        )
        original_response = self.compute_ik_proxy.call(bridged_request)
        response = self._melodic_get_position_ik.GetPositionIKResponse()
        response.solution = self._convert_melodic_robot_state_msg_to_noetic(
            original_response.solution
        )
        response.error_code = original_response.error_code
        return response

    def _get_planning_scene_srv_cb(self, request):
        bridged_request = self._melodic_get_planning_scene.GetPlanningSceneRequest()
        bridged_request.components = request.components
        original_response = self.get_planning_scene_proxy.call(bridged_request)
        response = self._melodic_get_planning_scene.GetPlanningSceneResponse()
        response.scene = self._convert_melodic_planning_scene_msg_to_noetic(
            original_response.scene
        )
        return response

    def _plan_kinematic_path_srv_cb(self, request):
        bridged_request = self._melodic_get_motion_plan.GetMotionPlanRequest()
        bridged_request.motion_plan_request = (
            self._convert_noetic_motion_plan_request_msg_to_melodic(
                request.motion_plan_request
            )
        )
        original_response = self.plan_kinematic_path_proxy.call(bridged_request)
        response = self._melodic_get_motion_plan.GetMotionPlanResponse()
        response.motion_plan_response = (
            self._convert_melodic_motion_plan_response_msg_to_noetic(
                original_response.motion_plan_response
            )
        )
        return response

    def _move_group_action_cb(self, goal):
        self.move_group_ac.send_goal(
            self._convert_noetic_move_group_goal_msg_to_melodic(goal),
            feedback_cb=self._move_group_feedback_cb,
        )
        self.move_group_ac.wait_for_result()
        result = self.move_group_ac.get_result()
        self.move_group_as.set_succeeded(
            self._convert_melodic_move_group_result_msg_to_noetic(result)
        )

    def _move_group_feedback_cb(self, feedback):
        self.move_group_as.publish_feedback(feedback)

    def _planning_scene_world_cb(self, msg):
        converted_msg = self._convert_noetic_planning_scene_world_msg_to_melodic(msg)
        self.planning_scene_world_pub.publish(converted_msg)

    def _convert_melodic_collision_object_msg_to_noetic(self, collision_object_msg):
        """
        :param collision_object_msg: Melodic CollisionObject message
        :type colision_object_msg: moveit_msgs.msg.CollisionObject
        :returns: Noetic CollisionObject message
        :rtype: moveit_msgs.msg.CollisionObject
        """
        noetic_msg = CollisionObject()
        noetic_msg.header = collision_object_msg.header
        noetic_msg.id = collision_object_msg.id
        noetic_msg.type = collision_object_msg.type
        noetic_msg.primitives = collision_object_msg.primitives
        noetic_msg.primitive_poses = collision_object_msg.primitive_poses
        noetic_msg.meshes = collision_object_msg.meshes
        noetic_msg.mesh_poses = collision_object_msg.mesh_poses
        noetic_msg.planes = collision_object_msg.planes
        noetic_msg.plane_poses = collision_object_msg.plane_poses
        noetic_msg.operation = collision_object_msg.operation
        # NOTE The field subframe_names, subframe_poses and pose are ignored
        return noetic_msg

    def _convert_noetic_collision_object_msg_to_melodic(self, collision_object_msg):
        """
        :param collision_object_msg: Noetic CollisionObject message
        :type colision_object_msg: moveit_msgs.msg.CollisionObject
        :returns: Melodic CollisionObject message
        :rtype: moveit_msgs.msg.CollisionObject
        """
        melodic_msg = self._melodic_collision_object.CollisionObject()
        melodic_msg.header = collision_object_msg.header
        melodic_msg.id = collision_object_msg.id
        melodic_msg.type = collision_object_msg.type
        melodic_msg.primitives = collision_object_msg.primitives
        melodic_msg.primitive_poses = collision_object_msg.primitive_poses
        melodic_msg.meshes = collision_object_msg.meshes
        melodic_msg.mesh_poses = collision_object_msg.mesh_poses
        melodic_msg.planes = collision_object_msg.planes
        melodic_msg.plane_poses = collision_object_msg.plane_poses
        melodic_msg.operation = collision_object_msg.operation
        # NOTE The field subframe_names, subframe_poses and pose are ignored
        return melodic_msg

    def _convert_melodic_attached_collision_object_msg_to_noetic(
        self, attached_collision_object_msg
    ):
        """
        :param attached_collision_object_msg: Melodic AttachedCollisionObject message
        :type attached_colision_object_msg: moveit_msgs.msg.AttachedCollisionObject
        :returns: Noetic AttachedCollisionObject message
        :rtype: moveit_msgs.msg.AttachedCollisionObject
        """
        noetic_msg = AttachedCollisionObject()
        noetic_msg.link_name = attached_collision_object_msg.link_name
        noetic_msg.object = self._convert_melodic_collision_object_msg_to_noetic(
            attached_collision_object_msg.object
        )
        noetic_msg.touch_links = attached_collision_object_msg.touch_links
        noetic_msg.detach_posture = attached_collision_object_msg.detach_posture
        noetic_msg.weight = attached_collision_object_msg.weight
        return noetic_msg

    def _convert_noetic_attached_collision_object_msg_to_melodic(
        self, attached_collision_object_msg
    ):
        """
        :param attached_collision_object_msg: Noetic AttachedCollisionObject message
        :type attached_colision_object_msg: moveit_msgs.msg.AttachedCollisionObject
        :returns: Melodic AttachedCollisionObject message
        :rtype: moveit_msgs.msg.AttachedCollisionObject
        """
        melodic_msg = self._melodic_attached_collision_object.AttachedCollisionObject()
        melodic_msg.link_name = attached_collision_object_msg.link_name
        melodic_msg.object = self._convert_noetic_collision_object_msg_to_melodic(
            attached_collision_object_msg.object
        )
        melodic_msg.touch_links = attached_collision_object_msg.touch_links
        melodic_msg.detach_posture = attached_collision_object_msg.detach_posture
        melodic_msg.weight = attached_collision_object_msg.weight
        return melodic_msg

    def _convert_noetic_motion_plan_request_msg_to_melodic(
        self, motion_plan_request_msg
    ):
        """
        :param motion_plan_request_msg: Noetic MotionPlanRequest message
        :type motion_plan_request_msg: moveit_msgs.msg.MotionPlanRequest
        :returns: Noetic MotionPlanRequest message
        :rtype: moveit_msgs.msg.MotionPlanRequest
        """
        melodic_msg = self._melodic_motion_plan_request.MotionPlanRequest()
        melodic_msg.workspace_parameters = motion_plan_request_msg.workspace_parameters
        melodic_msg.start_state = self._convert_noetic_robot_state_msg_to_melodic(
            motion_plan_request_msg.start_state
        )
        melodic_msg.goal_constraints = motion_plan_request_msg.goal_constraints
        melodic_msg.path_constraints = motion_plan_request_msg.path_constraints
        melodic_msg.trajectory_constraints = (
            motion_plan_request_msg.trajectory_constraints
        )
        melodic_msg.planner_id = motion_plan_request_msg.planner_id
        melodic_msg.group_name = motion_plan_request_msg.group_name
        melodic_msg.num_planning_attempts = (
            motion_plan_request_msg.num_planning_attempts
        )
        melodic_msg.allowed_planning_time = (
            motion_plan_request_msg.allowed_planning_time
        )
        melodic_msg.max_velocity_scaling_factor = (
            motion_plan_request_msg.max_velocity_scaling_factor
        )
        melodic_msg.max_acceleration_scaling_factor = (
            motion_plan_request_msg.max_acceleration_scaling_factor
        )
        return melodic_msg

    def _convert_melodic_motion_plan_response_msg_to_noetic(
        self, motion_plan_response_msg
    ):
        noetic_msg = MotionPlanResponse()
        noetic_msg.trajectory_start = self._convert_melodic_robot_state_msg_to_noetic(
            motion_plan_response_msg.trajectory_start
        )
        noetic_msg.group_name = motion_plan_response_msg.group_name
        noetic_msg.trajectory = motion_plan_response_msg.trajectory
        noetic_msg.planning_time = motion_plan_response_msg.planning_time
        noetic_msg.error_code = motion_plan_response_msg.error_code

    def _convert_noetic_move_group_goal_msg_to_melodic(self, move_group_goal_msg):
        melodic_msg = self._melodic_move_group_goal.MoveGroupGoal()
        melodic_msg.request = self._convert_noetic_motion_plan_request_msg_to_melodic(
            move_group_goal_msg.request
        )
        melodic_msg.planning_options.planning_scene_diff = (
            self._convert_noetic_planning_scene_msg_to_melodic(
                move_group_goal_msg.planning_options.planning_scene_diff
            )
        )
        melodic_msg.planning_options.plan_only = (
            move_group_goal_msg.planning_options.plan_only
        )
        melodic_msg.planning_options.look_around = (
            move_group_goal_msg.planning_options.look_around
        )
        melodic_msg.planning_options.look_around_attempts = (
            move_group_goal_msg.planning_options.look_around_attempts
        )
        melodic_msg.planning_options.max_safe_execution_cost = (
            move_group_goal_msg.planning_options.max_safe_execution_cost
        )
        melodic_msg.planning_options.replan = (
            move_group_goal_msg.planning_options.replan
        )
        melodic_msg.planning_options.replan_attempts = (
            move_group_goal_msg.planning_options.replan_attampts
        )
        melodic_msg.planning_options.replan_delay = (
            move_group_goal_msg.planning_options.replan_delay
        )
        return melodic_msg

    def _convert_melodic_move_group_result_msg_to_noetic(self, move_group_result_msg):
        noetic_msg = MoveGroupResult()
        noetic_msg.error_code = move_group_result_msg.error_code
        noetic_msg.trajectory_start = self._convert_melodic_robot_state_msg_to_noetic(
            move_group_result_msg.trajectory_start
        )
        noetic_msg.planned_trajectory = move_group_result_msg.planned_trajectory
        noetic_msg.executed_trajectory = move_group_result_msg.executed_trajectory
        noetic_msg.planning_time = move_group_result_msg.planning_time
        return noetic_msg

    def _convert_noetic_position_ik_request_msg_to_melodic(
        self, position_ik_request_msg
    ):
        """
        :param position_ik_request: Noetic PositionIKRequest message
        :type position_ik_request: moveit_msgs.msg.PositionIKRequest
        :returns: Melodic PositionIKRequest message
        :rtype: moveit_msgs.msg.PositionIKRequest
        """
        melodic_msg = self._melodic_position_ik_request.PositionIKRequest()
        melodic_msg.group_name = position_ik_request_msg.group_name
        melodic_msg.robot_state = self._convert_noetic_robot_state_msg_to_melodic(
            position_ik_request_msg.robot_state
        )
        melodic_msg.constraints = position_ik_request_msg.constraints
        melodic_msg.avoid_collisios = position_ik_request_msg.avoid_collisions
        melodic_msg.ik_link_name = position_ik_request_msg.ik_link_name
        melodic_msg.pose_stamped = position_ik_request_msg.pose_stamped
        melodic_msg.ik_link_names = position_ik_request_msg.ik_link_names
        melodic_msg.pose_stamped_vector = position_ik_request_msg.pose_stamped_vector
        melodic_msg.timeout = position_ik_request_msg.timeout
        melodic_msg.attempts = position_ik_request_msg.attempts
        return melodic_msg

    def _convert_melodic_planning_scene_msg_to_noetic(self, planning_scene_msg):
        """
        :param planning_scene_msg: Melodic PlanningScene message
        :type planning_scene_msg: moveit_msgs.msg.PlanningScene
        :returns: Noetic PlanningScene message
        :rtype: moveit_msgs.msg.PlanningScene
        """
        noetic_msg = PlanningScene()
        noetic_msg.name = planning_scene_msg.name
        noetic_msg.robot_state = self._convert_melodic_robot_state_msg_to_noetic(
            planning_scene_msg.robot_state
        )
        noetic_msg.robot_model_name = planning_scene_msg.robot_model_name
        noetic_msg.fixed_frame_transforms = planning_scene_msg.fixed_frame_transforms
        noetic_msg.allowed_collision_matrix = (
            planning_scene_msg.allowed_collision_matrix
        )
        noetic_msg.link_padding = planning_scene_msg.link_padding
        noetic_msg.link_scale = planning_scene_msg.link_scale
        noetic_msg.object_colors = planning_scene_msg.object_colors
        noetic_msg.world = self._convert_melodic_planning_scene_world_msg_to_noetic(
            planning_scene_msg.world
        )
        noetic_msg.is_diff = planning_scene_msg.is_diff
        return noetic_msg

    def _convert_noetic_planning_scene_msg_to_melodic(self, planning_scene_msg):
        """
        :param planning_scene_msg: Noetic PlanningScene message
        :type planning_scene_msg: moveit_msgs.msg.PlanningScene
        :returns: Melodic PlanningScene message
        :rtype: moveit_msgs.msg.PlanningScene
        """
        melodic_msg = self._melodic_planning_scene.PlanningScene()
        melodic_msg.name = planning_scene_msg.name
        melodic_msg.robot_state = self._convert_noetic_robot_state_msg_to_melodic(
            planning_scene_msg.robot_state
        )
        melodic_msg.robot_model_name = planning_scene_msg.robot_model_name
        melodic_msg.fixed_frame_transforms = planning_scene_msg.fixed_frame_transforms
        melodic_msg.allowed_collision_matrix = (
            planning_scene_msg.allowed_collision_matrix
        )
        melodic_msg.link_padding = planning_scene_msg.link_padding
        melodic_msg.link_scale = planning_scene_msg.link_scale
        melodic_msg.object_colors = planning_scene_msg.object_colors
        melodic_msg.world = self._convert_noetic_planning_scene_world_msg_to_melodic(
            planning_scene_msg.world
        )
        melodic_msg.is_diff = planning_scene_msg.is_diff
        return melodic_msg

    def _convert_melodic_planning_scene_world_msg_to_noetic(
        self, planning_scene_world_msg
    ):
        """
        :param planning_scene_world_msg: Melodic PlanningSceneWorld message
        :type planning_scene_world_msg: moveit_msgs.msg.PlanningSceneWorld
        :returns: Noetic PlanningSceneWorld message
        :rtype: moveit_msgs.msg.PlanningSceneWorld
        """
        noetic_msg = PlanningSceneWorld()
        for obj in planning_scene_world_msg.collision_objects:
            noetic_msg.collision_objects.append(
                self._convert_melodic_collision_object_msg_to_noetic(obj)
            )
        noetic_msg.octomap = planning_scene_world_msg.octomap
        return noetic_msg

    def _convert_noetic_planning_scene_world_msg_to_melodic(
        self, planning_scene_world_msg
    ):
        """
        :param planning_scene_world_msg: Noetic PlanningSceneWorld message
        :type planning_scene_world_msg: moveit_msgs.msg.PlanningSceneWorld
        :returns: Melodic PlanningSceneWorld message
        :rtype: moveit_msgs.msg.PlanningSceneWorld
        """
        melodic_msg = self._melodic_planning_scene_world.PlanningSceneWorld()
        for obj in planning_scene_world_msg.collision_objects:
            melodic_msg.collision_objects.append(
                self._convert_noetic_collision_object_msg_to_melodic(obj)
            )
        melodic_msg.octomap = planning_scene_world_msg.octomap
        return melodic_msg

    def _convert_melodic_robot_state_msg_to_noetic(self, robot_state_msg):
        """
        :param robot_state_msg: Melodic RobotState message
        :type robot_state_msg: moveit_msgs.msg.RobotState
        :returns: Noetic RobotState message
        :rtype: moveit_msgs.msg.RobotState
        """
        noetic_msg = RobotState()
        noetic_msg.joint_state = robot_state_msg.joint_state
        noetic_msg.multi_dof_joint_state = robot_state_msg.multi_dof_joint_state
        for obj in robot_state_msg.attached_collision_objects:
            noetic_msg.attached_collision_objects.append(
                self._convert_melodic_attached_collision_object_msg_to_noetic(obj)
            )
        noetic_msg.is_diff = robot_state_msg.is_diff
        return noetic_msg

    def _convert_noetic_robot_state_msg_to_melodic(self, robot_state_msg):
        """
        :param robot_state_msg: Noetic RobotState message
        :type robot_state_msg: moveit_msgs.msg.RobotState
        :returns: Melodic RobotState message
        :rtype: moveit_msgs.msg.RobotState
        """
        melodic_msg = self._melodic_robot_state.RobotState()
        melodic_msg.joint_state = robot_state_msg.joint_state
        melodic_msg.multi_dof_joint_state = robot_state_msg.multi_dof_joint_state
        melodic_msg.is_diff = robot_state_msg.is_diff
        for obj in robot_state_msg.attached_collision_objects:
            melodic_msg.attached_collision_objects.append(
                self._convert_noetic_attached_collision_object_msg_to_melodic(obj)
            )
        return melodic_msg


def main():
    r = rospkg.RosPack()
    moveit_msgs_manifest = r.get_manifest("moveit_msgs")
    current_version = moveit_msgs_manifest.version
    if version.parse(current_version) < version.parse("0.11.0"):
        rospy.logfatal("Unsupported moveit_msgs version: {}".format(current_version))
        return
    rospy.init_node("moveit_noetic_bridge")
    bridge_node = MoveitNoeticBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
