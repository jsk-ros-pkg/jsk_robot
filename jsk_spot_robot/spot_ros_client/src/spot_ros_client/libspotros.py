import PyKDL
import rospy
import actionlib

import math

# msg
from geometry_msgs.msg import Pose, Twist, Quaternion
# services
from std_srvs.srv import Trigger, TriggerRequest
from std_srvs.srv import SetBool, SetBoolRequest
from spot_msgs.srv import ListGraph, ListGraphRequest
from spot_msgs.srv import SetLocalizationFiducial, SetLocalizationFiducialRequest
from spot_msgs.srv import SetLocalizationWaypoint, SetLocalizationWaypointRequest
from spot_msgs.srv import SetLocomotion, SetLocomotionRequest
from spot_msgs.srv import UploadGraph, UploadGraphRequest
from spot_msgs.srv import Dock, DockRequest
# actions
from spot_msgs.msg import NavigateToAction, NavigateToGoal
from spot_msgs.msg import TrajectoryAction, TrajectoryGoal
from spot_behavior_manager_msgs.msg import LeadPersonAction, LeadPersonGoal


class SpotRosClient:

    def __init__(self,
                topicname_cmd_vel='/spot/cmd_vel',
                topicname_body_pose='/spot/body_pose',
                servicename_claim='/spot/claim',
                servicename_release='/spot/release',
                servicename_stop='/spot/stop',
                servicename_self_right='/spot/self_right',
                servicename_sit='/spot/sit',
                servicename_stand='/spot/stand',
                servicename_power_on='/spot/power_on',
                servicename_power_off='/spot/power_off',
                servicename_estop_hard='/spot/estop/hard',
                servicename_estop_gentle='/spot/estop/gentle',
                servicename_stair_mode='/spot/stair_mode',
                servicename_locomotion_mode='/spot/locomotion_mode',
                servicename_upload_graph='/spot/upload_graph',
                servicename_list_graph='/spot/list_graph',
                servicename_set_localization_fiducial='/spot/set_localization_fiducial',
                servicename_set_localization_waypoint='/spot/set_localization_waypoint',
                servicename_dock='/spot/dock',
                servicename_undock='/spot/undock',
                actionname_navigate_to='/spot/navigate_to',
                actionname_trajectory='/spot/trajectory',
                actionname_execute_behaviors='/spot_behavior_manager_server/execute_behaviors',
                duration_timeout=0.05):

        # Publishers
        self._pub_cmd_vel = rospy.Publisher(
                                    topicname_cmd_vel,
                                    Twist,
                                    queue_size=1
                                    )
        self._pub_body_pose = rospy.Publisher(
                                    topicname_body_pose,
                                    Pose,
                                    queue_size=1
                                    )

        # wait for services
        try:
            rospy.wait_for_service(servicename_claim, rospy.Duration(5))
            rospy.wait_for_service(servicename_release, rospy.Duration(5))
            rospy.wait_for_service(servicename_stop, rospy.Duration(5))
            rospy.wait_for_service(servicename_self_right, rospy.Duration(5))
            rospy.wait_for_service(servicename_sit, rospy.Duration(5))
            rospy.wait_for_service(servicename_stand, rospy.Duration(5))
            rospy.wait_for_service(servicename_power_on, rospy.Duration(5))
            rospy.wait_for_service(servicename_power_off, rospy.Duration(5))
            rospy.wait_for_service(servicename_estop_hard, rospy.Duration(5))
            rospy.wait_for_service(servicename_estop_gentle, rospy.Duration(5))
            rospy.wait_for_service(servicename_stair_mode, rospy.Duration(5))
            rospy.wait_for_service(servicename_locomotion_mode, rospy.Duration(5))
            rospy.wait_for_service(servicename_upload_graph, rospy.Duration(5))
            rospy.wait_for_service(servicename_list_graph, rospy.Duration(5))
            rospy.wait_for_service(servicename_set_localization_fiducial, rospy.Duration(5))
            rospy.wait_for_service(servicename_set_localization_waypoint, rospy.Duration(5))
            rospy.wait_for_service(servicename_dock, rospy.Duration(5))
            rospy.wait_for_service(servicename_undock, rospy.Duration(5))
        except rospy.ROSException as e:
            rospy.logerr('Service unavaliable: {}'.format(e))

        # Service Clients
        self._srv_client_claim = rospy.ServiceProxy(
                                    servicename_claim,
                                    Trigger
                                    )
        self._srv_client_release = rospy.ServiceProxy(
                                    servicename_release,
                                    Trigger
                                    )
        self._srv_client_stop = rospy.ServiceProxy(
                                    servicename_stop,
                                    Trigger
                                    )
        self._srv_client_self_right = rospy.ServiceProxy(
                                    servicename_self_right,
                                    Trigger
                                    )
        self._srv_client_sit = rospy.ServiceProxy(
                                    servicename_sit,
                                    Trigger
                                    )
        self._srv_client_stand = rospy.ServiceProxy(
                                    servicename_stand,
                                    Trigger
                                    )
        self._srv_client_power_on = rospy.ServiceProxy(
                                    servicename_power_on,
                                    Trigger
                                    )
        self._srv_client_power_off = rospy.ServiceProxy(
                                    servicename_power_off,
                                    Trigger
                                    )
        self._srv_client_estop_hard = rospy.ServiceProxy(
                                    servicename_estop_hard,
                                    Trigger
                                    )
        self._srv_client_estop_gentle = rospy.ServiceProxy(
                                    servicename_estop_gentle,
                                    Trigger
                                    )
        self._srv_client_stair_mode = rospy.ServiceProxy(
                                    servicename_stair_mode,
                                    SetBool
                                    )
        self._srv_client_locomotion_mode = rospy.ServiceProxy(
                                    servicename_locomotion_mode,
                                    SetLocomotion
                                    )
        self._srv_client_upload_graph = rospy.ServiceProxy(
                                    servicename_upload_graph,
                                    UploadGraph
                                    )
        self._srv_client_list_graph = rospy.ServiceProxy(
                                    servicename_list_graph,
                                    ListGraph
                                    )
        self._srv_client_set_localization_fiducial = rospy.ServiceProxy(
                                    servicename_set_localization_fiducial,
                                    SetLocalizationFiducial
                                    )
        self._srv_client_set_localization_waypoint = rospy.ServiceProxy(
                                    servicename_set_localization_waypoint,
                                    SetLocalizationWaypoint
                                    )
        self._srv_client_dock = rospy.ServiceProxy(
                                    servicename_dock,
                                    Dock
                                    )
        self._srv_client_undock = rospy.ServiceProxy(
                                    servicename_undock,
                                    Trigger
                                    )


        # Action Clients
        self._actionclient_navigate_to = actionlib.SimpleActionClient(
                                                actionname_navigate_to,
                                                NavigateToAction
                                                )
        self._actionclient_trajectory = actionlib.SimpleActionClient(
                                                actionname_trajectory,
                                                TrajectoryAction
                                                )
        self._actionclient_execute_behaviors = actionlib.SimpleActionClient(
                                                actionname_execute_behaviors,
                                                LeadPersonAction
                                                )

        # wait for action
        try:
            self._actionclient_navigate_to.wait_for_server(rospy.Duration(5))
            self._actionclient_trajectory.wait_for_server(rospy.Duration(5))
            self._actionclient_execute_behaviors.wait_for_server(rospy.Duration(5))
        except rospy.ROSException as e:
            rospy.logerr('Action unavaliable: {}'.format(e))


    def pubCmdVel(self, vx, vy, vtheta):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = vtheta
        self._pub_cmd_vel.publish(msg)

    def pubBodyPose(self, height, orientation):
        msg = Pose()
        msg.position.z = height
        msg.orientation = orientation
        self._pub_body_pose.publish(msg)

    def claim(self):
        res = self._srv_client_claim(TriggerRequest())
        return res.success, res.message

    def release(self):
        res = self._srv_client_release(TriggerRequest())
        return res.success, res.message

    def stop(self):
        res = self._srv_client_stop(TriggerRequest())
        return res.success, res.message

    def self_right(self):
        res = self._srv_client_self_right(TriggerRequest())
        return res.success, res.message

    def sit(self):
        res = self._srv_client_sit(TriggerRequest())
        return res.success, res.message

    def stand(self):
        res = self._srv_client_stand(TriggerRequest())
        return res.success, res.message

    def power_on(self):
        res = self._srv_client_power_on(TriggerRequest())
        return res.success, res.message

    def power_off(self):
        res = self._srv_client_power_off(TriggerRequest())
        return res.success, res.message

    def estop_hard(self):
        res = self._srv_client_estop_hard(TriggerRequest())
        return res.success, res.message

    def estop_gentle(self):
        res = self._srv_client_estop_gentle(TriggerRequest())
        return res.success, res.message

    def stair_mode(self, stair_mode):
        res = self._srv_client_stair_mode(SetBoolRequest(data=stair_mode))
        return res.success, res.message

    def locomotion_mode(self, locomotion_mode):
        res = self._srv_client_locomotion_mode(SetLocomotionRequest(locomotion_mode=locomotion_mode))
        return res.success, res.message

    def upload_graph(self, upload_filepath):
        res = self._srv_client_upload_graph(UploadGraphRequest(upload_filepath=upload_filepath))
        return res.success, res.message

    def list_graph(self):
        res = self._srv_client_list_graph(ListGraphRequest())
        return res.waypoint_ids

    def set_localization_fiducial(self):
        res = self._srv_client_set_localization_fiducial(SetLocalizationFiducialRequest())
        return res.success, res.message

    def set_localization_waypoint(self, waypoint_id):
        res = self._srv_client_set_localization_waypoint(SetLocalizationWaypointRequest(waypoint_id=waypoint_id))
        return res.success, res.message

    def dock(self, dock_id):
        req = TriggerRequest()
        req.dock_id = dock_id
        self.pubBodyPose(0,Quaternion(0,0,0,1.0))
        self.sit()
        self.stand()
        res = self._srv_client_dock(req)
        return res.success, res.message

    def undock(self):
        self.power_on()
        res = self._srv_client_undock(TriggerRequest())
        return res.success, res.message

    def navigate_to(self, id_navigate_to, blocking=False):
        goal = NavigateToGoal()
        goal.id_navigate_to = id_navigate_to
        self._actionclient_navigate_to.send_goal(goal)
        if blocking:
            self._actionclient_navigate_to.wait_for_result()
            return self._actionclient_navigate_to.get_result()

    def wait_for_navigate_to_result(self, duration=rospy.Duration(0)):
        return self._actionclient_navigate_to.wait_for_result(duration)

    def get_navigate_to_result(self):
        return self._actionclient_navigate_to.get_result()

    def cancel_navigate_to(self):
        self._actionclient_navigate_to.cancel_all_goals()

    def execute_behaviors(self, target_node_id, blocking=True):
        goal = LeadPersonGoal()
        goal.target_node_id = target_node_id
        self._actionclient_execute_behaviors.send_goal(goal)
        if blocking:
            self._actionclient_execute_behaviors.wait_for_result()
            return self._actionclient_execute_behaviors.get_result()

    def wait_execute_behaviors_result(self, duration=None):
        if duration is None:
            return self._actionclient_execute_behaviors.wait_for_result()
        else:
            return self._actionclient_execute_behaviors.wait_for_result(duration=duration)

    def get_execute_behaviors_result(self):
        return self._actionclient_execute_behaviors.get_result()

 ## \brief call trajectory service
 ##
 ##
 ## \param x x value of the target position [m]
 ## \param x y value of the target position [m]
 ## \param theta theta value of the target position [rad]
 ## \param duration duration of trajectory command [secs]
    def trajectory(self, x, y, theta, duration, blocking=False):
        rotation = PyKDL.Rotation.RotZ(theta)
        goal = TrajectoryGoal()
        goal.target_pose.header.frame_id = 'body'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = rotation.GetQuaternion()[0]
        goal.target_pose.pose.orientation.y = rotation.GetQuaternion()[1]
        goal.target_pose.pose.orientation.z = rotation.GetQuaternion()[2]
        goal.target_pose.pose.orientation.w = rotation.GetQuaternion()[3]
        goal.duration.data = rospy.Duration(duration)
        self._actionclient_trajectory.send_goal(goal)
        if blocking:
            self._actionclient_trajectory.wait_for_result()
