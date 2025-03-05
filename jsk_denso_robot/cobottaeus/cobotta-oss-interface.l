(require "package://cobottaeus/cobotta-utils.l")
(require "package://pr2eus/robot-interface.l")

(ros::load-ros-package "denso_cobotta_driver")
(ros::load-ros-package "denso_cobotta_gripper")
(ros::load-ros-package "std_msgs")

(defclass cobotta-oss-interface
  :super robot-interface
  :slots (gripper-action))

(defmethod cobotta-oss-interface
  (:init
    (&rest args)
    (prog1
      (send-super* :init :robot cobotta-robot
                   :joint-states-topic "/cobotta/joint_states"
                   :joint-states-queue-size 2  ;; joint_states of arm and gripper are separated
                   :groupname "cobotta_oss_interface" args)
      (send self :add-controller :rarm-controller)
      (ros::subscribe "/cobotta/gripper_state" std_msgs::Bool
                      #'send self :cobotta-gripper-state-callback
                      :groupname groupname)
      (setq gripper-action
            (instance ros::simple-action-client :init
                      "/cobotta/gripper_move"
                      denso_cobotta_gripper::GripperMoveAction
                      :groupname groupname))
      ;; /cobotta/gripper_action (using control_msgs::GripperCommandAction) also exists,
      ;; but it does not have speed input
      ))
  (:default-controller () (send self :rarm-controller))
  (:rarm-controller ()
    (list
      (list
        (cons :controller-action "cobotta/arm_controller/follow_joint_trajectory")
        (cons :controller-state "cobotta/arm_controller/state")
        (cons :action-type control_msgs::FollowJointTrajectoryAction)
        (cons :joint-names (list "joint_1" "joint_2" "joint_3" "joint_4" "joint_5" "joint_6")))))
  (:set-motor-state
    (state)
    "Set motor power ON/OFF.
When setting motor power OFF, brake becomes Lock.
Details: https://densorobot.github.io/docs/denso_cobotta_ros/command_reference/rosservice.html#motor
"
    (let ((srv "/cobotta/set_motor_state")
          (req (instance denso_cobotta_driver::SetMotorStateRequest :init :state state)))
      (if (ros::wait-for-service srv 0)
        (ros::service-call srv req)
        nil)))
  (:motor-on ()
    (send self :set-motor-state t))
  (:motor-off ()
    (send self :set-motor-state nil))
  (:get-motor-state ()
    (let ((srv "/cobotta/get_motor_state")
          (req (instance denso_cobotta_driver::GetMotorStateRequest :init)))
      (if (ros::wait-for-service srv 0)
        (ros::service-call srv req)
        nil)))
  (:set-brake-state
    (state)
    "Set brake Lock/Release.
Releasing brake after setting motor power OFF makes arm joints totally free.
Be careful of speed limit over, moving arm too fast makes brake locked again.
Details: https://densorobot.github.io/docs/denso_cobotta_ros/command_reference/rosservice.html#brake
"
    (let ((srv "/cobotta/set_brake_state")
          (req (instance denso_cobotta_driver::SetBrakeStateRequest :init :state state)))
      (if (ros::wait-for-service srv 0)
        (ros::service-call srv req)
        nil)))
  (:brake-lock ()
    (send self :set-brake-state (list t t t t t t)))
  (:brake-release ()
    (send self :set-brake-state (list nil nil nil nil nil nil)))
  (:get-brake-state ()
    (let ((srv "/cobotta/get_brake_state")
          (req (instance denso_cobotta_driver::GetBrakeStateRequest :init)))
      (if (ros::wait-for-service srv 0)
        (ros::service-call srv req)
        nil)))
  (:clear-error ()
    "Clear all errors except FATAL error.
If your robot is in emergency state, calling :motor-on after :clear-error probably returns your robot to normal state.
Details: https://densorobot.github.io/docs/denso_cobotta_ros/command_reference/rosservice.html#error
"
    (let ((srv "/cobotta/clear_error")
          (req (instance denso_cobotta_driver::ClearErrorRequest :init)))
      (if (ros::wait-for-service srv 0)
        (ros::service-call srv req)
        nil)))
  (:start-grasp
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp :pos 0 args)
    (send self :gripper :state))
  (:stop-grasp
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp :pos 0.015 args))
  (:go-grasp
    (&key (pos 0) (speed 50) (effort 10) (wait t))
    "Move gripper fingers to target `pos` with `speed` and `effort`.
Arguments:
- pos : target distance between each finger and the gripper center (0.0-0.015) [m]
- speed : movement speed (1.0-100.0) [%]
- effort : movement effort (6.0-20.0) [N]
- wait : if this argument is T, this method waits until the movement finishes
Details: https://densorobot.github.io/docs/denso_cobotta_ros/command_reference/rostopic.html#parallel-gripper
"
    (when (send self :simulation-modep)
      (send robot :joint_gripper :joint-angle (* pos 1000)) ;; m -> mm
      (send robot :joint_gripper_mimic :joint-angle (* pos -1000))
      (return-from :go-grasp t))
    (let (goal result)
      (setq goal (instance denso_cobotta_gripper::GripperMoveActionGoal :init))
      (send goal :goal :target_position pos)
      (send goal :goal :speed speed)
      (send goal :goal :effort effort)
      (send gripper-action :send-goal goal)
      (when wait (send gripper-action :wait-for-result))
      (setq result (send gripper-action :get-result))
      result))
  (:cobotta-gripper-state-callback
    (msg)
    (send self :set-robot-state1 :gripper-state (send msg :data)))
  (:gripper (key)
    "Get information about gripper
Arguments:
- key (:state)
Example: (send self :gripper :state) => t (hold something) or nil (not hold anything)
Details: https://github.com/DENSORobot/denso_cobotta_ros/blob/bb60e75adb8477ed3402561b4ec3ba687af3f397/denso_cobotta_lib/src/gripper.cpp#L47
"
   (if (send self :simulation-modep)
     (progn
       (ros::ros-warn ":gripper is not implemented on simulation mode, always returns t")
       t)
     ;; for real robot
     (send-super :state (intern (format nil "GRIPPER-~A" (string key)) *keyword-package*)))))

(defun cobotta-oss-init (&optional (create-viewer))
  (unless (boundp '*cobotta*) (cobotta) (send *cobotta* :reset-pose))
  (unless (ros::ok) (ros::roseus "cobotta_oss_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance cobotta-oss-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)

  (when create-viewer (objects (list *cobotta*))))
