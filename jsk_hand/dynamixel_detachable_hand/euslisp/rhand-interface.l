#!/usr/bin/env roseus
;; rhand client of the detachable hand (detachable-hand-interface.l, ROS 1 and ROS 2)

(unless (ros::ok)
  (ros::roseus "rhand_action_controller"))
(require "package://dynamixel_detachable_hand/euslisp/detachable-hand-interface.l")

(defparameter *rhand-detach-torque-constant* 2.0)
(defparameter *rhand-detach-current-ma* 500.0)
(defparameter *rhand-effort-limit* 0.50)
(defparameter *rhand-gripper-effort* 0.30)
(defparameter *rhand-gripper-close-effort* 0.50)
(defparameter *rhand-gripper-open-position* (detachable-hand-named-position "rhand" "gripper" "open"))
(defparameter *rhand-gripper-close-position* (detachable-hand-named-position "rhand" "gripper" "close"))
(defparameter *rhand-gripper-open-time* 0.25)

(defclass rhand-interface
  :super detachable-hand-interface
  :slots ())

(defmethod rhand-interface
  (:init (&rest args &key ((:groupname grp) "rhand") &allow-other-keys)
    (send-super* :init :side "rhand" :groupname grp args))
  (:detach-torque-constant () *rhand-detach-torque-constant*)
  (:detach-current-ma () *rhand-detach-current-ma*)
  (:effort-limit () *rhand-effort-limit*)
  (:gripper-effort () *rhand-gripper-effort*)
  (:gripper-close-effort () *rhand-gripper-close-effort*)
  (:gripper-open-position () *rhand-gripper-open-position*)
  (:gripper-close-position () *rhand-gripper-close-position*)
  (:gripper-open-time () *rhand-gripper-open-time*))

(defun rhand-init (&rest args)
  (if (not (boundp '*rhand*))
      (setq *rhand* (instance* rhand-interface :init args)))
  *rhand*)

(defun rhand (&key grasp (velocity 0.0) (acceleration 0.0) (effort 0.0))
  (unless (boundp '*rhand*) (rhand-init))
  (let ((grasp-angle
         (cond ((eq grasp 'open) 0.0)
               ((eq grasp 'close) -3.0)
               ((numberp grasp) (float grasp))
               (t 0.0))))
    (send *rhand* :move-hand grasp-angle :velocity velocity :acceleration acceleration :effort effort)))
