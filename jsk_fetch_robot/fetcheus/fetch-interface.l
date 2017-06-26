(ros::roseus "fetch")

(require "package://fetcheus/fetch-utils.l")
(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus_moveit/euslisp/robot-moveit.l")

(defclass fetch-interface
  :super robot-move-base-interface
  :slots (gripper-action moveit-robot)
  )

(defmethod fetch-interface
  (:init (&rest args)
   (prog1 (send-super* :init :robot fetch-robot :base-frame-id "/base_link" :odom-topic "/odom" :base-controller-action-name nil args)
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
  (:angle-vector
   (av &optional (tm 3000) &rest args) ;; (ctype controller-type) (start-time 0) &rest args
                                       ;;  &key (use-torso t) (clear-velocities t) &allow-other-keys)
   "Send joind angle to robot with self-collision motion planning, this method retuns immediately, so use :wait-interpolation to block until the motion stops.
- av : joint angle vector [rad]
- tm : time to goal in [msec]
- use-torso : set t to use torso
"
   (let ((ctype controller-type) (start-time 0) (use-torso t) (clear-velocities t))
   ;; as of 0.3.x, :angle-vector (robot-interface) :acceps tm ctype start-time as optional arguments, but in here we prefer old API
   (if (= (length args) 1) ;; args must be ctype
       (setq ctype (car args)
             args (cdr args)))
   (if (and (>= (length args) 2) (null (member (car args) '(:use-torso :start-time :clear-velocities))));; args must be ctype start-time
       (setq ctype (car args)
             start-time (cadr args)
             args (cddr args)))
   (if (member :use-torso args) (setq use-torso (cadr (member :use-torso args))))
   (if (member :start-time args) (setq use-torso (cadr (member :start-time args))))
   (if (member :clear-velocities args) (setq clear-velocities (cadr (member :clear-velocities args))))
   ;; for simulation mode
   (when (send self :simulation-modep)
     (return-from :angle-vector (send* self :angle-vector-raw av tm ctype start-time args)))
   ;;
   (when (not (numberp tm))
     (ros::warn ":angle-vector tm is not a number, use :angle-vector av tm args"))
   (send* self :angle-vector-motion-plan av :ctype ctype :move-arm :rarm :total-time tm
               :start-offset-time start-time :clear-velocities clear-velocities
               :use-torso use-torso args)))
  (:angle-vector-sequence
   (avs &optional tms &rest args) ;; (ctype controller-type) (start-time 0) &rest args
                                  ;; &key (use-torso t) (clear-velocities t) &allow-other-keys)
   "Send joind angle to robot with self-collision motion planning, this method returns immediately, so use :wait-interpolation to block until the motion stops.
- avs : sequence of joint angle vector [rad]
- tms : list of time to goal from previous angle-vector point in [msec]
- use-torso : set t to use torso
"
   (let ((ctype controller-type) (start-time 0) (use-torso t) (clear-velocities t))
   ;; as of 0.3.x, :angle-vector (robot-interface) :acceps tm ctype start-time as optional arguments, but in here we prefer old API
   (if (= (length args) 1) ;; args must be ctype
       (setq ctype (car args)
             args (cdr args)))
   (if (and (>= (length args) 2) (null (member (car args) '(:use-torso :start-time :clear-velocities))));; args must be ctype start-time
       (setq ctype (car args)
             start-time (cadr args)
             args (cddr args)))
   (if (member :use-torso args) (setq use-torso (cadr (member :use-torso args))))
   (if (member :start-time args) (setq use-torso (cadr (member :start-time args))))
   (if (member :clear-velocities args) (setq clear-velocities (cadr (member :clear-velocities args))))
   ;; for simulation mode
   (when (send self :simulation-modep)
     (return-from :angle-vector-sequence
                  (send* self :angle-vector-sequence-raw avs tms ctype start-time args)))
   (unless (and (listp tms) (every #'numberp tms))
     (ros::warn ":angle-vector-sequence tms is not a list of number, use :angle-vector-sequence av tms args"))
   (if tms
     (setq tms (apply #'+ tms))
     (setq tms 3000))
   (send* self :angle-vector-motion-plan avs :ctype ctype :move-arm :rarm :total-time tms
               :start-offset-time start-time :clear-velocities clear-velocities
               :use-torso use-torso args)))
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
    (setq effort (or effort 40))
    (let (goal result)
      (setq goal (instance control_msgs::GripperCommandActionGoal :init))
      (send goal :goal :command :position pos)
      (send goal :goal :command :max_effort effort)
      (send gripper-action :send-goal goal)
      (when wait (send gripper-action :wait-for-result))
      (setq result (send gripper-action :get-result))
      result))
  ;;
  (:speak (text &rest args)
    (let ()
      (apply #'speak-en text :topic-name "/sound_play" args)))
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
