import rospy

import actionlib

from softhand_ros.msg import CalibJointAction
from softhand_ros.msg import CalibJointResult

from dynamixel_msgs.msg import MotorStateList


class CalibRequiredController(object):
    def __init__(self):
        self.calib_speed = rospy.get_param(
            '{}/calib_speed'.format(self.controller_namespace), 0.1)
        self.calib_torque_limit = rospy.get_param(
            '{}/calib_torque_limit'.format(self.controller_namespace), 0.3)
        self.detect_limit_load = rospy.get_param(
            '{}/detect_limit_load'.format(self.controller_namespace), 0.15)
        self.is_multiturn = rospy.get_param(
            '{}/is_multiturn'.format(self.controller_namespace), False)

        self.calib_server = actionlib.SimpleActionServer(
            '{}/calib'.format(self.controller_namespace),
            CalibJointAction,
            execute_cb=self.on_calib_action,
            auto_start=False)

    def calib_initialize(self):
        self.__calib()
        self.calib_server.start()
        return (not rospy.is_shutdown())

    def on_calib_action(self, goal):
        self.__calib()
        self.calib_server.set_succeeded(CalibJointResult())

    def __calib(self):
        # Initialize joint position
        self.motor_states_for_init = {}
        motor_states_sub_for_init = rospy.Subscriber(
            'motor_states/{}'.format(self.port_namespace),
            MotorStateList, self.get_motor_states_for_init)
        self.set_speed(0.0)
        # Backup current angle limits
        prev_limits = self.__get_angle_limits()
        # Change to wheel mode
        self.__set_angle_limits(0, 0)
        self.set_torque_limit(self.calib_torque_limit)
        self.__set_speed_wheel(0.0)
        # release torque by disabling it
        self.set_torque_enable(False)
        rospy.sleep(0.2)
        self.set_torque_enable(True)  # re-enable it
        rospy.sleep(0.2)
        if self.flipped:
            self.__set_speed_wheel(self.calib_speed)
        else:
            self.__set_speed_wheel(-self.calib_speed)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                init_pos = self.motor_states_for_init['position']
                init_load = abs(self.motor_states_for_init['load'])
                if init_load > self.detect_limit_load:
                    break
            except KeyError:
                pass
            rate.sleep()
        self.__set_speed_wheel(0.0)
        if self.is_multiturn:
            # Change to multiturn mode
            self.__set_angle_limits(4095, 4095)
        else:
            # Change to previous mode
            self.__set_angle_limits(prev_limits['min'], prev_limits['max'])
        self.set_torque_enable(False)
        self.set_speed(self.joint_speed)
        rospy.sleep(0.5)
        if self.torque_limit is not None:
            self.set_torque_limit(self.torque_limit)
        motor_states_sub_for_init.unregister()

        # Remember initial joint position
        diff = init_pos - self.initial_position_raw
        self.initial_position_raw += diff
        rospy.set_param(
            '{}/motor/init'.format(self.controller_namespace),
            self.initial_position_raw)
        self.min_angle_raw += diff
        rospy.set_param(
            '{}/motor/min'.format(self.controller_namespace),
            self.min_angle_raw)
        self.max_angle_raw += diff
        rospy.set_param(
            '{}/motor/max'.format(self.controller_namespace),
            self.max_angle_raw)
        if self.flipped:
            min_angle = self.initial_position_raw - self.min_angle_raw
            max_angle = self.initial_position_raw - self.max_angle_raw
        else:
            min_angle = self.min_angle_raw - self.initial_position_raw
            max_angle = self.max_angle_raw - self.initial_position_raw
        self.min_angle = min_angle * self.RADIANS_PER_ENCODER_TICK
        self.max_angle = max_angle * self.RADIANS_PER_ENCODER_TICK

    def __set_angle_limits(self, min_angle, max_angle):
        self.dxl_io.set_angle_limits(self.motor_id, min_angle, max_angle)

    def __get_angle_limits(self):
        return self.dxl_io.get_angle_limits(self.motor_id)

    def get_motor_states_for_init(self, state_list):
        state = filter(
            lambda state: state.id == self.motor_id, state_list.motor_states)
        if state:
            state = state[0]
            self.motor_states_for_init['position'] = state.position
            self.motor_states_for_init['load'] = state.load

    def __spd_rad_to_raw_wheel(self, spd_rad):
        if spd_rad < -self.joint_max_speed:
            spd_rad = -self.joint_max_speed
        elif spd_rad > self.joint_max_speed:
            spd_rad = self.joint_max_speed
        return int(round(spd_rad / self.VELOCITY_PER_TICK))

    def __set_speed_wheel(self, speed):
        mcv = (self.motor_id, self.__spd_rad_to_raw_wheel(speed))
        self.dxl_io.set_multi_speed([mcv])
