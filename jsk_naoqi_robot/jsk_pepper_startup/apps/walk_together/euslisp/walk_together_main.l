#!/usr/bin/env roseus

(load "package://jsk_pepper_startup/apps/walk_together/euslisp/walk_together_utils.l")
(ros::load-ros-package "peppereus")

;; load simple model for fast-boot
(require :pepper "package://peppereus/pepper-simple.l")

;; load files
(require :pepper-interface "package://peppereus/pepper-interface.l")

(unless (boundp '*ri*)
  (pepper-init))
(objects (list *pepper*))
(if (not (string= (send *ri* :get-life) "disabled"))
    (send *ri* :disable-life))

(defparameter *continue* t)

(ros::roseus "walk_together_hand_in_hand")

(defun start-func (&rest args)
  (let ()
    (send *ri* :servo-on)
    (unix:sleep 1)
    (send *ri* :send-stiffness-controller :larm 0.0)
    (unix:sleep 1)
    (send *ri* :send-stiffness-controller :rarm 0.0)
    (send *ri* :set-language "Japanese")
    (unix:sleep 1)
    (send *ri* :set-external-collision-protection-status 1 nil)
    (unix:sleep 1)
    (send *ri* :set-external-collision-protection-status 2 nil)
    (send *ri* :speak "\\vct=130\\出発ッ！\\pau=100\\電源コードに気をつけてねッ")
    (send *ri* :show-image "walk_together_pepper_tablet_picture.png")
    :started))

(defun main-func (&rest args)
  (setq walk-together-utils-class (instance walk_together_utils :init))
  (let ((previous-moving-direction "init"))
    (while *continue*
      (let ((vec))
	(ros::spin-once)	    
	(setq vec (send walk-together-utils-class :get-moving-vec))
	(send walk-together-utils-class :set-moving-direction vec)
	(if (not (string= previous-moving-direction (send walk-together-utils-class :get-moving-direction)))
	    (send walk-together-utils-class :look-at (send walk-together-utils-class :get-moving-direction)))
	(send walk-together-utils-class :rotate (send walk-together-utils-class :get-moving-direction))
	(send walk-together-utils-class :go-forward (send walk-together-utils-class :get-moving-direction))
	(if (and
	     (send walk-together-utils-class :get-moving-direction)
	     (not (string= previous-moving-direction (send walk-together-utils-class :get-moving-direction))))
	    (setq previous-moving-direction (send walk-together-utils-class :get-moving-direction))
	  )
	(ros::duration-sleep 0.2)
	))
    (unless *continue*
      (return-from main-func :finished))
    :next))

(defun end-func (&rest args)
  (let ()
    (send *ri* :speak "\\vct=130\\到着ッ")
    (send *ri* :set-external-collision-protection-status 1 t)
    (unix:sleep 1)
    (send *ri* :set-external-collision-protection-status 2 t)
    (unix:sleep 1)
    (send *ri* :speak "\\vct=130\\お散歩モード終了中\\pau=500\\ちょっと待っててねッ")
    (send *ri* :hide-image)
    (unix:sleep 1)
    (send *ri* :send-stiffness-controller :larm 1.0)
    (unix:sleep 2)
    (send *ri* :send-stiffness-controller :rarm 1.0)
    (unix:sleep 2)
    (send *ri* :servo-on)
    (send *ri* :speak "\\vct=130\\お散歩モード終了ッ")
    :finished))

(load "package://roseus_smach/src/state-machine-ros.l")
(defun walking-together-sm ()
  (let (sm)
    (setq sm
	  (make-state-machine
	   '((:start :started :main)    ;; transitions (node transition node)
	     (:main  :next :main)
	     (:main  :finished :end)
	     (:end   :finished :goal)
	     )
	   '((:start 'start-func)       ;; node-to-function maps
	     (:end   'end-func)
	     (:main  'main-func)
	     )
	   '(:start)                    ;; initial node
	   '(:goal)                     ;; goal node
	   ))
    (send sm :arg-keys 'description)
    sm))

(defun ros::roseus-sigint-handler (sig code)
  (ros::ros-warn (format nil "ros::roseus-sigint-handler ~A" sig))
  (setq *continue* nil))
(unix:signal unix::sigint 'ros::roseus-sigint-handler)

(exec-state-machine (walking-together-sm))
(exit)
