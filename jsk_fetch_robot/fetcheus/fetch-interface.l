(ros::roseus "fetch")

(require "package://fetcheus/fetch.l")
(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus_moveit/euslisp/robot-moveit.l")

(defclass fetch-interface
  :super robot-move-base-interface
  :slots (gripper-action moveit-robot)
  )

(defmethod fetch-interface
  (:init (&rest args)
   (prog1 (send-super* :init :robot fetch-robot :base-frame-id "/base_link" :odom-topic "/odom" args)
     (send self :add-controller :arm-controller)
     (send self :add-controller :torso-controller)
     (send self :add-controller :head-controller)
     (setq gripper-action
           (instance ros::simple-action-client :init
                     "/gripper_controller/gripper_action"
                     control_msgs::GripperCommandAction
                     :groupname groupname))
     (setq moveit-robot (instance fetch-robot :init))
     (send self :set-moveit-environment (instance fetch-moveit-environment :init :robot moveit-robot))
     ))
  (:angle-vector-raw (&rest args) (send-super* :angle-vector args))
  (:angle-vector-sequence-raw (&rest args) (send-super* :angle-vector-sequence args))
  (:angle-vector ;; this verison uses :angle-vector-sequence for sendding trajectory, this enable us to use :wait-interpolation method so we choose this for now
   (av &optional (tm 3000) &rest args)
   "Send joind angle to robot with self-collision motion planning, this method retuns immediately, so use :wait-interpolation to block until the motion stops.
- av : joint angle vector [rad]
- tm : (time to goal in [msec]) ;; currently this value is ignored
- use-torso : set t to use torso
"
   (let ((use-torso nil))
     (if (cadr (member :use-torso args)) (setq use-torso t))
     (send self :angle-vector-motion-plan av :move-arm :rarm :use-torso use-torso)
   ))
  ;;
  (:default-controller ()
   (append
    (send self :arm-controller)
    (send self :torso-controller)
    (send self :head-controller)))
  (:arm-controller ()
   (list
    (list
     (cons :controller-action "/arm_controller/follow_joint_trajectory")
     (cons :controller-state "/arm_controller/follow_joint_trajectory/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "shoulder_pan_joint" "shoulder_lift_joint" "upperarm_roll_joint" "elbow_flex_joint" "forearm_roll_joint" "wrist_flex_joint" "wrist_roll_joint")))))
  (:torso-controller ()
   (list
    (list
     (cons :controller-action "/torso_controller/follow_joint_trajectory")
     (cons :controller-state "/torso_controller/follow_joint_trajectory/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "torso_lift_joint")))))
  (:head-controller ()
   (list
    (list
     (cons :controller-action "/head_controller/follow_joint_trajectory")
     (cons :controller-state "/head_controller/follow_joint_trajectory/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "head_pan_joint" "head_tilt_joint")))))
  (:start-grasp
    (&key (effort) (wait t))
    (send self :go-grasp :pos 0 :effort effort :wait wait))
  (:stop-grasp
    (&key (effort) (wait t))
    (send self :go-grasp :pos 0.1 :effort effort :wait wait))
  (:go-grasp
    (&key (pos 0) (effort) (wait t))
    (setq effort (or effort 20))
    (let (goal result)
      (setq goal (instance control_msgs::GripperCommandActionGoal :init))
      (send goal :goal :command :position pos)
      (send goal :goal :command :max_effort effort)
      (send gripper-action :send-goal goal)
      (when wait (send gripper-action :wait-for-result))
      (setq result (send gripper-action :get-result))
      result))
  ;;
  )

(defun fetch-init (&optional (create-viewer))
  (unless (boundp '*fetch*) (fetch) (send *fetch* :reset-pose))
  (unless (ros::ok) (ros::roseus "fetch_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance fetch-interface :init)))
  
  (ros::spin-once)
  (send *ri* :spin-once)
  
  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (when create-viewer (objects (list *fetch*)))
  )

(defclass fetch-moveit-environment
  :super moveit-environment)
(defmethod fetch-moveit-environment
  (:init (&key ((:robot rb) *fetch*) &rest args)
         (send-super* :init :robot rb :frame-id "base_link" args))
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "arm")
               (cons :target-link
                     (send self :search-link-from-name "wrist_roll_link"))
               (cons :joint-list (send robot :rarm :joint-list))
               )
         (list :rarm-torso
               (cons :group-name "arm_with_torso")
               (cons :target-link
                     (send self :search-link-from-name "wrist_roll_link"))
               (cons :joint-list (append
                                  (send robot :torso :joint-list)
                                  (send robot :rarm :joint-list)))
               )
#|
    <group name="arm">
        <joint name="shoulder_pan_joint" />
        <joint name="shoulder_lift_joint" />
        <joint name="upperarm_roll_joint" />
        <joint name="elbow_flex_joint" />
        <joint name="forearm_roll_joint" />
        <joint name="wrist_flex_joint" />
        <joint name="wrist_roll_joint" />
    </group>
    <group name="arm_with_torso">
        <joint name="torso_lift_joint" />
        <joint name="shoulder_pan_joint" />
        <joint name="shoulder_lift_joint" />
        <joint name="upperarm_roll_joint" />
        <joint name="elbow_flex_joint" />
        <joint name="forearm_roll_joint" />
        <joint name="wrist_flex_joint" />
        <joint name="wrist_roll_joint" />
    </group>
    <!--END EFFECTOR: Purpose: Represent information about an end effector.-->
    <end_effector name="gripper" parent_link="wrist_roll_link" group="gripper" />
|#
         ))
  )

#|
(setq *ri* (instance fetch-interface :init))
(setq *fetch* (fetch))
(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
|#
