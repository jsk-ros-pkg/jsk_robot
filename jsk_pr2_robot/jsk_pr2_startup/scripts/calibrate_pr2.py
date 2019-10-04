#!/usr/bin/python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import getopt
import sys

import actionlib
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from pr2_controllers_msgs.srv import QueryCalibrationState
from pr2_mechanism_msgs.srv import ListControllers
from pr2_mechanism_msgs.srv import LoadController
from pr2_mechanism_msgs.srv import SwitchController
from pr2_mechanism_msgs.srv import SwitchControllerRequest
from pr2_mechanism_msgs.srv import UnloadController
import roslib
import rospy
from sensor_msgs.msg import JointState
from sound_play.msg import SoundRequest
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_srvs.srv import Empty
import yaml


calibration_params_namespace = "calibration_controllers"
load_controller = rospy.ServiceProxy(
    'pr2_controller_manager/load_controller',
    LoadController)
unload_controller = rospy.ServiceProxy(
    'pr2_controller_manager/unload_controller',
    UnloadController)
switch_controller = rospy.ServiceProxy(
    'pr2_controller_manager/switch_controller',
    SwitchController)
list_controllers = rospy.ServiceProxy(
    'pr2_controller_manager/list_controllers',
    ListControllers)

hold_position = {'r_shoulder_pan': -0.7,
                 'l_shoulder_pan': 0.7,
                 'r_elbow_flex': -2.0,
                 'l_elbow_flex': -2.0,
                 'r_upper_arm_roll': 0.0,
                 'l_upper_arm_roll': 0.0,
                 'r_shoulder_lift': 1.0,
                 'l_shoulder_lift': 1.0}

force_calibration = False


def get_controller_name(joint_name):
    return calibration_params_namespace + "/calibrate/cal_%s" % joint_name


def set_force_calibration(joint_name):
    rospy.set_param(
        calibration_params_namespace +
        "/calibrate/cal_%s/force_calibration" %
        joint_name,
        True)


def get_holding_name(joint_name):
    return "%s/hold/%s_position_controller" % (
        calibration_params_namespace, joint_name)


def get_service_name(joint_name):
    return '%s/is_calibrated' % get_controller_name(joint_name)

global last_joint_states
last_joint_states = None


def joint_states_cb(msg):
    global last_joint_states
    last_joint_states = msg
rospy.Subscriber('joint_states', JointState, joint_states_cb)

global motors_halted
motors_halted = None


def motor_state_cb(msg):
    global motors_halted
    motors_halted = msg.data
    rospy.logdebug("motors halted = %d" % motors_halted)
rospy.Subscriber('pr2_ethercat/motors_halted', Bool, motor_state_cb)

pub_diag = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)


def diagnostics(level, msg_short, msg_long):
    if level == 0:
        rospy.loginfo(msg_long)
    elif level == 1:
        rospy.logwarn(msg_long)
    elif level == 2:
        rospy.logerr(msg_long)
    d = DiagnosticArray()
    d.header.stamp = rospy.Time.now()
    ds = DiagnosticStatus()
    ds.level = level
    ds.message = msg_long
    ds.name = msg_short
    d.status = [ds]
    pub_diag.publish(d)


class HoldingController(object):
    def __init__(self, joint_name):
        self.joint_name = joint_name
        rospy.logdebug(
            "Loading holding controller: %s" %
            get_holding_name(joint_name))
        resp = load_controller(get_holding_name(joint_name))
        if resp.ok:
            rospy.logdebug(
                "Starting holding controller for joint %s." %
                joint_name)
            switch_controller([get_holding_name(joint_name)],
                              [], SwitchControllerRequest.STRICT)
            self.pub_cmd = rospy.Publisher(
                "%s/command" %
                get_holding_name(joint_name),
                Float64,
                latch=True,
                queue_size=10)
        else:
            rospy.logerr(
                "Failed to load holding controller for joint %s." %
                joint_name)
            raise Exception('Failure to load holding controller')

    def __del__(self):
        switch_controller([],
                          [get_holding_name(self.joint_name)],
                          SwitchControllerRequest.STRICT)
        unload_controller(get_holding_name(self.joint_name))
        self.pub_cmd.unregister()

    def hold(self, position):
        self.pub_cmd.publish(Float64(position))


class StatusPub(object):
    def __init__(self):
        self.pub_status = rospy.Publisher(
            'calibration_status', String, queue_size=10)
        self.status = {}
        self.status['active'] = []
        self.status['done'] = []

    def publish(self, active=None):
        if active:
            self.status['active'] = active
        else:
            self.status['done'].extend(self.status['active'])
            self.status['active'] = []

        str = "====\n"
        str += "Calibrating: %s\n" % ", ".join(self.status["active"])
        str += "Calibrated: %s\n" % ", ".join(self.status["done"])
        self.pub_status.publish(str)


class CalibrateParallel(object):
    def __init__(self, joints, status):
        self.joints = []
        self.hold_controllers = []
        self.services = {}
        self.status = status

        self.actionclient = actionlib.SimpleActionClient(
            '/robotsound_jp',
            SoundRequestAction)
        self.actionclient.wait_for_server()

        # spawn calibration controllers for all joints
        for j in joints:
            rospy.logdebug("Loading controller: %s" % get_controller_name(j))
            global force_calibration
            if force_calibration:
                set_force_calibration(j)
            resp = load_controller(get_controller_name(j))
            if resp.ok:
                # get service call to calibration controller to check
                # calibration state
                rospy.logdebug("Waiting for service: %s" % get_service_name(j))
                rospy.wait_for_service(get_service_name(j))
                self.services[j] = rospy.ServiceProxy(
                    get_service_name(j), QueryCalibrationState)
                self.joints.append(j)
            else:
                rospy.logerr(
                    "Failed to load calibration for joint %s."
                    'Skipping this joint' % j)

    def __del__(self):
        # stop controllers that were started
        switch_controller([],
                          [get_controller_name(j) for j in self.joints],
                          SwitchControllerRequest.BEST_EFFORT)

        # kill controllers that were loaded
        for j in self.joints:
            unload_controller(get_controller_name(j))

    def is_calibrated(self):
        # check if joints are calibrated
        for j in self.joints:
            if self.services[j]().is_calibrated:
                rospy.logdebug("joint %s is already calibrated" % j)
            else:
                rospy.logdebug("joint %s needs to be calibrated" % j)
                return False
        return True

    def calibrate(self):
        # start all calibration controllers
        rospy.logdebug("Start calibrating joints %s" % self.joints)
        switch_controller([get_controller_name(j)
                           for j in self.joints], [],
                          SwitchControllerRequest.STRICT)

        # wait for joints to calibrate
        self.status.publish(self.joints)
        start_time = rospy.Time.now()
        while not self.is_calibrated():
            if motors_halted and motors_halted == 1:
                msg = SoundRequest()
                msg.sound = SoundRequest.SAY
                msg.command = SoundRequest.PLAY_ONCE
                msg.arg = u"キャリブレーションをするためにランストップをオンにしてください"
                msg.volume = 1.0

                goal = SoundRequestGoal()
                goal.sound_request = msg
                self.actionclient.send_goal(goal)
                self.actionclient.wait_for_result()
                diagnostics(
                    2,
                    'Calibration on hold',
                    'Calibration is on hold because motors'
                    ' are halted. Enable the run-stop')
                start_time = rospy.Time.now()
                rospy.sleep(1.0)
            elif rospy.Time.now() > start_time + rospy.Duration(30.0):
                msg = SoundRequest()
                msg.sound = SoundRequest.SAY
                msg.command = SoundRequest.PLAY_ONCE
                msg.arg = u"%sがスタックしました。アシストしてください。" % self.joints
                msg.volume = 1.0

                goal = SoundRequestGoal()
                goal.sound_request = msg
                self.actionclient.send_goal(goal)
                self.actionclient.wait_for_result()

                # time for spine to go up is 29 seconds
                diagnostics(
                    2,
                    'Calibration stuck',
                    'Joint %s is taking a long time to calibrate. '
                    'It might be stuck and need some human help' %
                    self.joints)
                rospy.sleep(1.0)
            rospy.sleep(0.1)

        rospy.logdebug(
            "Stop calibration controllers for joints %s" %
            self.joints)
        switch_controller([],
                          [get_controller_name(j) for j in self.joints],
                          SwitchControllerRequest.BEST_EFFORT)

        # hold joints in place
        rospy.logdebug(
            "Loading holding controllers for joints %s" %
            self.joints)
        self.hold_controllers = []
        for j in self.joints:
            if j in hold_position:
                holder = HoldingController(j)
                holder.hold(hold_position[j])
                self.hold_controllers.append(holder)
        self.status.publish()


class CalibrateSequence(object):
    def __init__(self, sequence, status):
        self.status = status

        # create CalibrateParallel for all groups in sequence
        self.groups = []
        for s in sequence:
            self.groups.append(CalibrateParallel(s, status))

    def is_calibrated(self):
        # check if all groups in sequence are calibrated
        for g in self.groups:
            if not g.is_calibrated():
                return False
        return True

    def calibrate(self):
        # Check if this sequence needs calibration
        if self.is_calibrated():
            return True

        # calibrate all groups in sequence
        for g in self.groups:
            g.calibrate()


def main():
    try:
        rospy.init_node('calibration', anonymous=True, disable_signals=True)
        calibration_start_time = rospy.Time.now()

        rospy.wait_for_service('pr2_controller_manager/load_controller')
        rospy.wait_for_service('pr2_controller_manager/switch_controller')
        rospy.wait_for_service('pr2_controller_manager/unload_controller')

        # parse options
        allowed_flags = [
            'alpha-casters',
            'alpha-head',
            'alpha2b-head',
            'arms=',
            'force_calibration',
            'recalibrate']
        opts, args = getopt.gnu_getopt(rospy.myargv(), 'h', allowed_flags)
        caster_list = ['caster_fl', 'caster_fr', 'caster_bl', 'caster_br']
        head_list = ['head_pan', 'head_tilt']
        arms = 'auto'
        recalibrate = False
        global force_calibration
        for o, a in opts:
            if o == '-h':
                rospy.loginfo("Flags:", ' '.join(
                    ['--' + f for f in allowed_flags]))
                sys.exit(0)
            elif o == '--alpha-casters':
                caster_list = [
                    'caster_fl_alpha2',
                    'caster_fr_alpha2',
                    'caster_bl_alpha2',
                    'caster_br_alpha2']
            elif o == '--alpha-head':
                head_list = ['head_pan_alpha2', 'head_tilt']
            elif o == '--alpha2b-head':
                head_list = ['head_pan_alpha2', 'head_tilt_alpha2b']
            elif o == '--arms':
                arms = a
            elif o == '--force_calibration':
                force_calibration = True
            elif o == '--recalibrate':
                recalibrate = True
                force_calibration = True

        if arms not in ['both', 'none', 'left', 'right', 'auto']:
            print('Arms must be "both", "none", "left", "right", or "auto"')
            sys.exit(1)

        # load controller configuration
        rospy.loginfo(
            "Loading controller configuration on parameter server...")
        pr2_controller_configuration_dir = \
            roslib.packages.get_pkg_dir(
                'pr2_controller_configuration')
        calibration_yaml = '%s/pr2_calibration_controllers.yaml' \
            % pr2_controller_configuration_dir
        hold_yaml = '%s/pr2_joint_position_controllers.yaml' \
            % pr2_controller_configuration_dir
        if len(args) < 3:
            rospy.loginfo(
                "No yaml files specified for calibration "
                'and holding controllers, using defaults')
        else:
            calibration_yaml = args[1]
            hold_yaml = args[2]
        rospy.set_param(
            calibration_params_namespace +
            "/calibrate",
            yaml.load(
                open(calibration_yaml)))
        rospy.set_param(
            calibration_params_namespace +
            "/hold",
            yaml.load(
                open(hold_yaml)))

        # status publishing
        imustatus = True
        joints_status = False
        if not recalibrate:
            pub_calibrated = rospy.Publisher(
                'calibrated', Bool, latch=True, queue_size=10)
            pub_calibrated.publish(False)
        status = StatusPub()

        # Auto arm selection, determined based on which joints exist.
        if arms == 'auto':
            started_waiting = rospy.get_rostime()
            global last_joint_states
            while rospy.get_rostime() <= (started_waiting +
                                          rospy.Duration(50.0)):
                if last_joint_states:
                    break
            js = last_joint_states
            if not js:
                rospy.logwarn(
                    'Could not do auto arm selection '
                    'because no joint state was received')
                arms = 'both'
            else:
                if ('r_shoulder_pan_joint' in js.name and
                        'l_shoulder_pan_joint' in js.name):
                    arms = 'both'
                elif 'r_shoulder_pan_joint' in js.name:
                    arms = 'right'
                elif 'l_shoulder_pan_joint' in js.name:
                    arms = 'left'
                else:
                    arms = 'none'
            rospy.loginfo(
                "Arm selection was set to \"auto\".  "
                "Chose to calibrate using \"%s\"" %
                arms)

        # define calibration sequence objects
        torso = CalibrateSequence([['torso_lift']], status)
        gripper_list = []
        arm_list = []
        if arms == 'both':
            arm_list = [['r_shoulder_pan', 'l_shoulder_pan'],
                        ['r_elbow_flex', 'l_elbow_flex'],
                        ['r_upper_arm_roll', 'l_upper_arm_roll'],
                        ['r_shoulder_lift', 'l_shoulder_lift'],
                        ['r_forearm_roll',
                         'r_wrist',
                         'l_forearm_roll',
                         'l_wrist']]
            gripper_list = ['r_gripper', 'l_gripper']
        if arms == 'left':
            arm_list = [['l_shoulder_pan'],
                        ['l_elbow_flex'],
                        ['l_upper_arm_roll'],
                        ['l_shoulder_lift'],
                        ['l_forearm_roll', 'l_wrist']]
            gripper_list = ['l_gripper']
        if arms == 'right':
            arm_list = [['r_shoulder_pan'],
                        ['r_elbow_flex'],
                        ['r_upper_arm_roll'],
                        ['r_shoulder_lift'],
                        ['r_forearm_roll', 'r_wrist']]
            gripper_list = ['r_gripper']
        arm = CalibrateSequence(arm_list, status)
        gripper = CalibrateSequence([gripper_list], status)
        head = CalibrateSequence([head_list, ['laser_tilt']], status)
        caster = CalibrateSequence([caster_list], status)

        # if recalibrating, stop all running controllers first
        if recalibrate:
            controller_list = list_controllers()

            def is_running(c): return c[1] == 'running'
            running_controllers = [
                c[0] for c in filter(
                    is_running, zip(
                        controller_list.controllers, controller_list.state))]
            print("Running controllers : ", running_controllers)
            if not switch_controller(
                    [], running_controllers, SwitchControllerRequest.STRICT):
                print("Failed to stop controllers")
                sys.exit(1)

        # calibrate imu and torso
        if not torso.is_calibrated():
            rospy.loginfo('Calibrating imu')
            status.publish(['imu'])
            imu_calibrate_srv_name = 'torso_lift_imu/calibrate'
            rospy.wait_for_service(imu_calibrate_srv_name)
            imu_calibrate_srv = rospy.ServiceProxy(
                imu_calibrate_srv_name, Empty)
            try:
                imu_calibrate_srv()
                status.publish()
                # only calibrate torso when imu calibration succeeds
            except Exception:
                imustatus = False
            rospy.loginfo('Calibrating imu finished')
        else:
            rospy.loginfo('Not calibrating imu')

        # calibrate torso
        torso.calibrate()

        # calibrate arms
        torso_holder = None
        if not arm.is_calibrated():
            torso_holder = HoldingController('torso_lift')
            torso_holder.hold(0.25)
            rospy.sleep(5.0)
            rospy.loginfo('Moving up spine to allow arms to calibrate')
            arm.calibrate()
            rospy.loginfo('Moving down spine after arm calibration')
            torso_holder.hold(0.01)
            rospy.sleep(20.0)

        # calibrate rest of robot
        gripper.calibrate()
        head.calibrate()
        caster.calibrate()

        joints_status = True
        status.publish()

    finally:
        rospy.loginfo("Bringing down calibration node")

        rospy.set_param(calibration_params_namespace, "")
        del arm
        del gripper
        del torso
        del torso_holder
        del caster
        del head

        if not imustatus and not joints_status:
            rospy.logerr("Both mechanism and IMU calibration failed")
        elif not joints_status:
            rospy.logerr(
                "IMU calibration complete, but mechanism calibration failed")
        elif not imustatus:
            rospy.logerr(
                "Mechanism calibration complete, but IMU calibration failed.")
        else:
            rospy.loginfo('Calibration completed in %f sec' %
                          (rospy.Time.now() - calibration_start_time).to_sec())

        if recalibrate:
            if not switch_controller(
                    running_controllers, [], SwitchControllerRequest.STRICT):
                print("Could not start previously running controllers")

        if not recalibrate:
            if joints_status:
                pub_calibrated.publish(True)
            rospy.spin()


if __name__ == '__main__':
    main()
