#!/usr/bin/env roseus
(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :aero-arm "package://aero_description/models/aero-arm-upper.l")

(ros::load-ros-manifest "trajectory_msgs")
(ros::load-ros-manifest "pr2_controllers_msgs")


(defmethod AeroArmRobot-robot
  (:hand (&rest args)
    (if (null args) (return-from :hand nil))
    (let ((key (car args))
	  (nargs (cdr args))
	  )
      (unless (keywordp key) (return-from :hand nil))
      (case key
	    (:arms
	     (send self :hand-poses :rarm nargs)
	     )
	    (:rarm
	     (send self :hand-poses :rarm nargs))
	    (t
	     nil)
	    )
      ))
  (:hand-poses (hand args)
    (let ((pose (car args))
	  (angle 0)
	  (hand-vector (float-vector
			(send self hand :t-1p :joint-angle)
			(send self hand :f-1p :joint-angle)
			0 0))
	  )
      (case pose
	    (:reset-pose (setq hand-vector #f(0 0 0 0)))
	    (:open-pose (if (eq hand :larm) (setq hand-vector #f(-50 50 0 0))
			  (setq hand-vector #f(50 -50 0 0))))
	    (:close-pose (if (eq hand :larm) (setq hand-vector #f(50 -50 0 0))
			   (setq hand-vector #f(-50 50 0 0))))
	    (:joint-angle (progn
			    (if (eq (cdr args) nil)
				(return-from :hand-poses
					     (* (if (eq hand :larm) 1 -1)
						(send self hand :t-1p :joint-angle))))
			    (setq angle (car (cdr args)))
			    (if (eq hand :larm)
				(setq hand-vector
				      (float-vector angle (- angle) 0 0))
			      (setq hand-vector
				    (float-vector (- angle) angle 0 0))
			      )
			    ))
	    (t (return-from :hand-poses nil))
	    )
      (send self hand :t-1p :joint-angle (elt hand-vector 0))
      (send self hand :f-1p :joint-angle (elt hand-vector 1))
      (send self hand :f-2p :joint-angle (elt hand-vector 2))
      (send self hand :f-3p :joint-angle (elt hand-vector 3))
      hand-vector))
  )



(defclass aero-upper-interface
  :super rtm-ros-robot-interface
  :slots (actual-state
	  desired-state))
(defmethod aero-upper-interface
  (:init (&rest args)
    (send-super* :init :robot AeroArmRobot-robot args)
    (mapcar #'(lambda (ctype)
                (send self :add-controller ctype))
            (send self :default-controller-list))
    (ros::roseus "aero_upper_interface")
    (ros::advertise "/aero_controller/command" trajectory_msgs::JointTrajectory)
    (ros::subscribe "/aero_controller/state"
		    pr2_controllers_msgs::JointTrajectoryControllerState
		    #'send self :controller-callback 1)
    )
  (:default-controller-list ()
   (list :rarm-controller
         :fullbody-controller)
   )
  (:default-controller ()
    (mapcar
     #'(lambda (ctype) (car (send self ctype))) (send self :default-controller-list))
    )
  (:fullbody-controller ()
   (send-message self robot-interface :default-controller)
   )
  (:rarm-controller ()
   (list
    (list
     (cons :controller-action "/rarm_controller/follow_joint_trajectory_action")
     (cons :controller-state  "/rarm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names
	   (list "J1" "J2" "J3" "J4" "J5" "J6"))
     ))
   )

  (:angle-vector (av &optional (tm 5000))
    (let ((joint-names (list "r_shoulder_y_joint"
                             "r_shoulder_p_joint"
			     "r_elbow_joint"
			     "r_wrist_y_joint"
			     "r_wrist_p_joint"
			     "r_wrist_r_joint"
			     "r_thumb_joint"
                             ))
	  (send-av (float-vector
		    (deg2rad (elt av 0))
		    (deg2rad (elt av 1))
		    (deg2rad (elt av 2))
		    (deg2rad (elt av 3))
		    (deg2rad (elt av 4))
		    (deg2rad (elt av 5))
		    (deg2rad (elt av 6))
                    ))
	  )
      (ros::publish "/aero_controller/command"
	  (instance trajectory_msgs::JointTrajectory
	     :init
	     :joint_names joint-names
	     :points (list (instance trajectory_msgs::JointTrajectoryPoint
			      :init
			      :positions send-av
			      :time_from_start (ros::time (/ tm 1000.0)))
			   ))
	  )
     av))
  (:get-angle-vector (av)
    (float-vector
     (rad2deg (cdr (assoc "r_shoulder_y_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_shoulder_p_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_elbow_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_wrist_y_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_wrist_p_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_wrist_r_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_thumb_joint" av :test #'equal)) )
     0
     0
     0
     ))
  (:reference-vector (&optional (element nil))
     (ros::spin-once)
     (if (eq element nil)
	 (send self :get-angle-vector desired-state)
       (elt (send self :get-angle-vector desired-state) element) )
     )
  (:actual-vector (&optional (element nil))
     (ros::spin-once)
     (if (eq element nil)
	 (send self :get-angle-vector actual-state)
       (elt (send self :get-angle-vector actual-state) element) )
     )
  (:controller-callback (msg)
     (let ((i 0)
	   )
       (setq actual-state '())
       (setq desired-state '())
       (dolist (x (send msg :joint_names))
	 (setq actual-state
	       (cons (cons x (elt (send msg :actual :positions) i)) actual-state))
	 (setq desired-state
	       (cons (cons x (elt (send msg :desired :positions) i)) desired-state))
	 (incf i))
       ))

  );; end of defmethod aero-arm-robot



(defun aero-init (&rest args)
  (when (not (boundp '*ri*))
    (let ((disp (copy-object x::*display*))
	  )
      (setf x::*display* 0)
      (setq *ri* (instance* aero-upper-interface :init args))
      (setf x::*display* disp)
      ))
  (if (not (boundp '*aero*))
      (setq *aero* (instance AeroArmRobot-robot :init))
    )
  (setq ((send *aero* :rarm :elbow-p) . max-angle) 180)
  (send *aero* :angle-vector (send *ri* :actual-vector))
  )

(defun load-controllers ()
  (load "package://aeroeus/aero-wheels.l")
  (load "package://aeroeus/aero-hand.l")
  (load "package://aeroeus/aero-torso.l")
  )

(defvar *base-time* 1000)
(defvar *real* nil)
