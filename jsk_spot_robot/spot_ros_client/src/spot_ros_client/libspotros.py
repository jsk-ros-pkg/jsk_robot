import PyKDL
import rospy
import actionlib

import math

# msg
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String
from spot_msgs.msg import Feedback
from spot_msgs.msg import PowerState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
# services
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerRequest
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolRequest
from spot_msgs.srv import ListGraph
from spot_msgs.srv import ListGraphRequest
from spot_msgs.srv import SetLocalizationFiducial
from spot_msgs.srv import SetLocalizationFiducialRequest
from spot_msgs.srv import SetLocalizationWaypoint
from spot_msgs.srv import SetLocalizationWaypointRequest
from spot_msgs.srv import SetLocomotion
from spot_msgs.srv import SetLocomotionRequest
from spot_msgs.srv import UploadGraph
from spot_msgs.srv import UploadGraphRequest
from spot_msgs.srv import Dock
from spot_msgs.srv import DockRequest
# actions
from spot_behavior_manager_msgs.msg import LeadPersonAction
from spot_behavior_manager_msgs.msg import LeadPersonGoal
from spot_msgs.msg import NavigateToAction
from spot_msgs.msg import NavigateToGoal
from spot_msgs.msg import TrajectoryAction
from spot_msgs.msg import TrajectoryGoal


def calc_distance_to_pose(pose):
    if isinstance(pose, Pose):
        return math.sqrt(pose.position.x ** 2 + pose.position.y ** 2 + pose.position.z ** 2)
    elif isinstance(pose, PoseStamped):
        pose = pose.pose
        return math.sqrt(pose.position.x ** 2 + pose.position.y ** 2 + pose.position.z ** 2)
    else:
        raise TypeError(
            'pose must be geometry_msgs.msg.Pose or geometry_msgs.msg.PoseStamped')


def convert_msg_point_to_kdl_vector(point):
    return PyKDL.Vector(point.x, point.y, point.z)


def get_nearest_person_pose(topicname='/spot_recognition_person_tracker/people_pose_array'):
    try:
        msg = rospy.wait_for_message(
            topicname,
            PoseArray,
            timeout=rospy.Duration(5))
    except rospy.ROSException as e:
        rospy.logwarn('Timeout exceede: {}'.format(e))
        return None

    if len(msg.poses) == 0:
        rospy.logwarn('No person visible')
        return None

    distance = calc_distance_to_pose(msg.poses[0])
    target_pose = msg.poses[0]
    for pose in msg.poses:
        if calc_distance_to_pose(pose) < distance:
            distance = calc_distance_to_pose(pose)
            target_pose = pose

    pose_stamped = PoseStamped()
    pose_stamped.header = msg.header
    pose_stamped.pose = target_pose

    return pose_stamped


def get_diff_for_person(pose):

    if isinstance(pose, Pose):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
    elif isinstance(pose, PoseStamped):
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
    else:
        raise TypeError(
            'pose must be geometry_msgs.msg.Pose or geometry_msgs.msg.PoseStamped')

    yaw = math.atan2(y, x)
    try:
        pitch = math.acos(z / math.sqrt(x**2 + y**2))
    except ValueError:
        pitch = 0
    return pitch, yaw


class SpotRosClient:

    def __init__(self,
                 topicname_cmd_vel='/spot/cmd_vel',
                 topicname_body_pose='/spot/body_pose',
                 topicname_cable_connected='/spot/status/cable_connected',
                 topicname_current_node_id='/spot_behavior_manager_server/current_node_id',
                 topicname_laptop_percentage='/spot/status/laptop_battery_percentage',
                 topicname_battery_percentage='/spot/status/battery_percentage',
                 topicname_status_feedback='/spot/status/feedback',
                 topicname_status_power_state='/spot/status/power_state',
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

        self.topicname_cable_connected = topicname_cable_connected
        self.topicname_current_node_id = topicname_current_node_id
        self.topicname_laptop_percentage = topicname_laptop_percentage
        self.topicname_battery_percentage = topicname_battery_percentage
        self.topicname_status_feedback = topicname_status_feedback
        self.topicname_status_power_state = topicname_status_power_state

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
            rospy.wait_for_service(
                servicename_locomotion_mode, rospy.Duration(5))
            rospy.wait_for_service(servicename_upload_graph, rospy.Duration(5))
            rospy.wait_for_service(servicename_list_graph, rospy.Duration(5))
            rospy.wait_for_service(
                servicename_set_localization_fiducial, rospy.Duration(5))
            rospy.wait_for_service(
                servicename_set_localization_waypoint, rospy.Duration(5))
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
            self._actionclient_execute_behaviors.wait_for_server(
                rospy.Duration(5))
        except rospy.ROSException as e:
            rospy.logerr('Action unavaliable: {}'.format(e))

    def get_laptop_percepntage(self):
        try:
            msg = rospy.wait_for_message(
                self.topicname_laptop_percentage, Float32, timeout=rospy.Duration(5))
            return msg.data
        except rospy.ROSException as e:
            rospy.logwarn('{}'.format(e))
            return None

    def get_battery_percepntage(self):
        try:
            msg = rospy.wait_for_message(
                self.topicname_battery_percentage, Float32, timeout=rospy.Duration(5))
            return msg.data
        except rospy.ROSException as e:
            rospy.logwarn('{}'.format(e))
            return None

    def is_connected(self):
        try:
            msg = rospy.wait_for_message(
                self.topicname_cable_connected, Bool, timeout=rospy.Duration(5))
            return msg.data
        except rospy.ROSException as e:
            rospy.logwarn('{}'.format(e))
            return None

    def is_sitting(self):
        try:
            msg = rospy.wait_for_message(
                self.topicname_status_feedback, Feedback, timeout=rospy.Duration(5))
            return msg.sitting
        except rospy.ROSException as e:
            rospy.logwarn('{}'.format(e))
            return None

    def is_standing(self):
        try:
            msg = rospy.wait_for_message(
                self.topicname_status_feedback, Feedback, timeout=rospy.Duration(5))
            return msg.standing
        except rospy.ROSException as e:
            rospy.logwarn('{}'.format(e))
            return None

    def is_powered_on(self):
        try:
            msg = rospy.wait_for_message(
                self.topicname_status_power_state, PowerState, timeout=rospy.Duration(5))
            if msg.motor_power_state == PowerState.STATE_ON:
                return True
            else:
                return False
        except rospy.ROSException as e:
            rospy.logwarn('{}'.format(e))
            return None

    def pub_cmd_vel(self, vx, vy, vtheta):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = vtheta
        self._pub_cmd_vel.publish(msg)

    def pub_body_pose(self, height, orientation):
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
        res = self._srv_client_locomotion_mode(
            SetLocomotionRequest(locomotion_mode=locomotion_mode))
        return res.success, res.message

    def upload_graph(self, upload_filepath):
        res = self._srv_client_upload_graph(
            UploadGraphRequest(upload_filepath=upload_filepath))
        return res.success, res.message

    def list_graph(self):
        res = self._srv_client_list_graph(ListGraphRequest())
        return res.waypoint_ids

    def set_localization_fiducial(self):
        res = self._srv_client_set_localization_fiducial(
            SetLocalizationFiducialRequest())
        return res.success, res.message

    def set_localization_waypoint(self, waypoint_id):
        res = self._srv_client_set_localization_waypoint(
            SetLocalizationWaypointRequest(waypoint_id=waypoint_id))
        return res.success, res.message

    def dock(self, dock_id):
        req = DockRequest()
        req.dock_id = dock_id
        self.pub_body_pose(0, Quaternion(0, 0, 0, 1.0))
        self.sit()
        self.stand()
        res = self._srv_client_dock(req)
        return res.success, res.message

    def undock(self):
        self.power_on()
        res = self._srv_client_undock(TriggerRequest())
        if not self.is_powered_on():
            self.power_on()
        if not self.is_standing():
            self.stand()
        return res.success, res.message

    def auto_dock(self, dock_id, home_node_id='eng2_73B2'):
        res = self.execute_behaviors(home_node_id)
        if not res.success:
            return res.success, res.message
        success, message = self.dock(dock_id)
        return success, message

    def auto_undock(self):
        self.sit()
        self.power_off()
        self.release()
        self.claim()
        self.power_on()
        if self.is_connected():
            self.undock()
        else:
            self.stand()

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

    def get_current_node(self):
        try:
            msg = rospy.wait_for_message(
                self.topicname_current_node_id, String, timeout=rospy.Duration(5))
            return msg.data
        except rospy.ROSException as e:
            rospy.logwarn('{}'.format(e))
            return None

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

    # \brief call trajectory service
    # \param x x value of the target position [m]
    # \param x y value of the target position [m]
    # \param theta theta value of the target position [rad]
    # \param duration duration of trajectory command [secs]
    def trajectory(self, x, y, theta, duration=None, blocking=False):
        rotation = PyKDL.Rotation.RotZ(theta)
        goal = TrajectoryGoal()
        goal.target_pose.header.frame_id = 'body'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = rotation.GetQuaternion()[0]
        goal.target_pose.pose.orientation.y = rotation.GetQuaternion()[1]
        goal.target_pose.pose.orientation.z = rotation.GetQuaternion()[2]
        goal.target_pose.pose.orientation.w = rotation.GetQuaternion()[3]
        if duration is None:
            goal.duration.data = rospy.Duration(math.sqrt(x**2 + y**2)*10)
        else:
            goal.duration.data = rospy.Duration(duration)
        self._actionclient_trajectory.send_goal(goal)
        if blocking:
            self._actionclient_trajectory.wait_for_result()
