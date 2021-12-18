#!/usr/bin/env roseus

(require "package://pr2eus/robot-interface.l")
(load "package://mycoboteus/mycobot-utils.l")


(defclass mycobot-interface
  :super robot-interface
  :slots ()
  )


(defmethod mycobot-interface
  (:init (&rest args)
    (prog1 (send-super* :init :robot mycobot-robot :joint-states-topic "joint_states" args)
      (send self :add-controller :rarm-controller)
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
              (list "joint1" "joint2" "joint3" "joint4" "joint5" "joint6"))))))


(defun mycobot-init ()
  (setq *mycobot* (instance mycobot-robot :init))
  (setq *robot* *mycobot*)
  (setq *ri* (instance mycobot-interface :init))
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (objects (list *robot*)))


;; send *mycobot* :inverse-kinematics (make-coords :pos #f(240 0 100)) :rotation-axis nil) :move-target (send *mycobot* :rarm :end-coords)
;; send *mycobot* :rarm :inverse-kinematics (make-coords :pos #f(240 0 100)) :rotation-axis nil)
