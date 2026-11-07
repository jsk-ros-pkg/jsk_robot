#!/usr/bin/env roseus
;; lhand client of the detachable hand (detachable-hand-interface.l, ROS 1 and ROS 2)

(unless (ros::ok)
  (ros::roseus "lhand_action_controller"))
(require "package://detachable_tools/euslisp/detachable-hand-interface.l")

(defparameter *lhand-detach-torque-constant* 1.15)
(defparameter *lhand-detach-current-ma* 400.0)
(defparameter *lhand-effort-limit* 0.50)
(defparameter *lhand-gripper-effort* 0.50)
(defparameter *lhand-gripper-close-effort* 0.50)
(defparameter *lhand-gripper-open-position* (detachable-hand-named-position "lhand" "gripper" "open"))
(defparameter *lhand-gripper-close-position* (detachable-hand-named-position "lhand" "gripper" "close"))
(defparameter *lhand-gripper-open-time* 0.25)

(defclass lhand-interface
  :super detachable-hand-interface
  :slots ())

(defmethod lhand-interface
  (:init (&rest args &key ((:groupname grp) "lhand") &allow-other-keys)
    (send-super* :init :side "lhand" :groupname grp args))
  (:detach-torque-constant () *lhand-detach-torque-constant*)
  (:detach-current-ma () *lhand-detach-current-ma*)
  (:effort-limit () *lhand-effort-limit*)
  (:gripper-effort () *lhand-gripper-effort*)
  (:gripper-close-effort () *lhand-gripper-close-effort*)
  (:gripper-open-position () *lhand-gripper-open-position*)
  (:gripper-close-position () *lhand-gripper-close-position*)
  (:gripper-open-time () *lhand-gripper-open-time*))

(defun lhand-init (&rest args)
  (if (not (boundp '*lhand*))
      (setq *lhand* (instance* lhand-interface :init args)))
  *lhand*)

(defun lhand (&key grasp (velocity 0.0) (acceleration 0.0) (effort 0.0))
  (unless (boundp '*lhand*) (lhand-init))
  (let ((grasp-angle
         (cond ((eq grasp 'open) 0.0)
               ((eq grasp 'close) -3.0)
               ((numberp grasp) (float grasp))
               (t 0.0))))
    (send *lhand* :move-hand grasp-angle :velocity velocity :acceleration acceleration :effort effort)))
