#!/usr/bin/env roseus

(require "package://pr2eus/robot-interface.l")
(require "package://pr2eus_moveit/euslisp/robot-moveit.l")
(load "package://mycoboteus/mycobot-utils.l")
(ros::roseus-add-srvs "std_srvs")

(defclass mycobot-moveit-environment
  :super moveit-environment)

(defmethod mycobot-moveit-environment
  (:init (&key args)
         (send-super* :init :robot (instance mycobot-robot :init) :frame-id "link1" args))
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "arm_group")
               (cons :target-link
                     (send self :search-link-from-name "link7"))
               (cons :joint-list (send robot :rarm :joint-list)))))
  )


(defclass mycobot-interface
  :super robot-interface
  :slots ()
  )


(defmethod mycobot-interface
  (:init (&rest args)
    (prog1 (send-super* :init :robot mycobot-robot :joint-states-topic "joint_states" args)
      (send self :add-controller :rarm-controller)
      (setq gripper-action
            (instance ros::simple-action-client :init
                      "/gripper_controller/gripper_command"
                      control_msgs::GripperCommandAction
                      :groupname groupname))
      (send self :set-moveit-environment (instance mycobot-moveit-environment :init))
    ))
  (:default-controller ()
    (append
      (send self :rarm-controller)))
  (:rarm-controller ()
    (list
      (list
        (cons :controller-action "/arm_controller/follow_joint_trajectory")
        (cons :controller-state "/arm_controller/follow_joint_trajectory/state")
        (cons :action-type control_msgs::FollowJointTrajectoryAction)
        (cons :joint-names
              (list "joint1" "joint2" "joint3" "joint4" "joint5" "joint6")))))

  (:go-grasp (&key (pos 0) (wait t))
    (when (send self :simulation-modep)
      ;; TODO: add mimic joint
      (return-from :go-grasp t))
    ;; we believe the low-level algorithm of mycobot can perform graspping with proper effort.
    ;; so we do not need to assign a specific posiiton for gripper.
    (let (goal result)
      (setq goal (instance control_msgs::GripperCommandActionGoal :init))
      (send goal :goal :command :position pos)
      (send gripper-action :send-goal goal)
      (when wait (send gripper-action :wait-for-result))
      (setq result (send gripper-action :get-result))
      result))
  (:start-grasp
   (&rest args &key &allow-other-keys)
   (send* self :go-grasp :pos 1 args)) ;; 1 denote to grasp
  (:stop-grasp
   (&rest args &key &allow-other-keys)
   (send* self :go-grasp :pos 0 args)) ;; 0 denote to grasp
  (:servo-on
   "Servo On all joints."
   (ros::service-call
    "/set_servo" (instance std_srvs::SetBoolRequest :init :data t)))
  (:servo-off
   "Servo Off all joints."
   (ros::service-call
    "/set_servo" (instance std_srvs::SetBoolRequest :init :data nil)))
  )


(defun mycobot-init ()
  (setq *mycobot* (instance mycobot-robot :init))
  (setq *robot* *mycobot*)
  (setq *ri* (instance mycobot-interface :init))
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (objects (list *robot*)))


;; send *mycobot* :inverse-kinematics (make-coords :pos #f(240 0 100)) :rotation-axis nil) :move-target (send *mycobot* :rarm :end-coords)
;; send *mycobot* :rarm :inverse-kinematics (make-coords :pos #f(240 0 100)) :rotation-axis nil)
