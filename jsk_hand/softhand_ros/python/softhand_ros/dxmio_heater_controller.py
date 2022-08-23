import threading

import rospy

# from dynamixel_driver.dynamixel_const import *

from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

# Control Table Constants for DXMIO
DXMIO_PWM_DUTY_0 = 22
DXMIO_PWM_DUTY_1 = 24
DXMIO_PWM_DUTY_2 = 26


class DxmioHeaterController(object):
    # initialize, start, stop
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        self.motor_id = rospy.get_param(
            self.controller_namespace + '/motor/id')
        self.joint_name = rospy.get_param(
            self.controller_namespace + '/joint_name')
        self.lock = threading.Lock()

        # joint_state
        self.joint_state = JointState(
            name=self.joint_name, motor_ids=[self.motor_id])

    def initialize(self):
        return True

    def start(self):
        self.running = True
        self.joint_state_pub = rospy.Publisher(
            self.controller_namespace + '/state', JointState, queue_size=1)
        self.command1_sub = rospy.Subscriber(
            self.controller_namespace + '/command1',
            Float64, self.process_command1)
        self.command2_sub = rospy.Subscriber(
            self.controller_namespace + '/command2',
            Float64, self.process_command2)
        self.command3_sub = rospy.Subscriber(
            self.controller_namespace + '/command3',
            Float64, self.process_command3)

    def stop(self):
        self.running = False
        self.joint_state_pub.unregister()
        self.motor_states_sub.unregister()
        self.command_sub.unregister()
        self.speed_service.shutdown('normal shutdown')
        self.torque_service.shutdown('normal shutdown')
        self.compliance_slope_service.shutdown('normal shutdown')

    def set_pwm_duty(self, address, pwm_duty):
        sid = self.motor_id
        # PWM duty limit for circuit protection
        pwm_duty = min(pwm_duty, 0.1)
        # PWM duty conversion 1.0 -> 65535
        pwm_duty = int(65535 * pwm_duty)
        pwm_duty &= 0xffff
        # split pwm_duty into 2 bytes
        lo_val = int(pwm_duty % 256)
        hi_val = int(pwm_duty >> 8)
        # use write to broadcast multi servo message
        self.dxl_io.write(sid, address, (lo_val, hi_val))
        rospy.loginfo('address: {}'.format(address))
        rospy.loginfo('pwm_duty: {}'.format(pwm_duty))

    def process_command1(self, msg):
        with self.lock:
            self.set_pwm_duty(DXMIO_PWM_DUTY_0, msg.data)
            rospy.loginfo('command1 done')
            # rospy.sleep(1.0)

    def process_command2(self, msg):
        with self.lock:
            self.set_pwm_duty(DXMIO_PWM_DUTY_1, msg.data)
            rospy.loginfo('command2 done')
            # rospy.sleep(1.0)

    def process_command3(self, msg):
        with self.lock:
            self.set_pwm_duty(DXMIO_PWM_DUTY_2, msg.data)
            rospy.loginfo('command3 done')
            # rospy.sleep(1.0)
