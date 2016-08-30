#!/usr/bin/env roseus
(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :aero-upper "package://aero_description/models/aero_upper.l")

(ros::load-ros-manifest "trajectory_msgs")
(ros::load-ros-manifest "pr2_controllers_msgs")

(defmethod AeroUpperRobot-robot
  (:hand (&rest args)
    (if (null args) (return-from :hand nil))
    (let ((key (car args))
	  (nargs (cdr args))
	  )
      (unless (keywordp key) (return-from :hand nil))
      (case key
	    (:arms
	     (send self :hand-poses :larm nargs)
	     (send self :hand-poses :rarm nargs)
	     )
	    (:larm
	     (send self :hand-poses :larm nargs))
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
	  desired-state
	  object-l
	  object-r
	  base-coords))
(defmethod aero-upper-interface
  (:init (&rest args)
    (send-super* :init :robot AeroUpperRobot-robot args)
    (mapcar #'(lambda (ctype)
                (send self :add-controller ctype))
            (send self :default-controller-list))
    (ros::roseus "aero_upper_interface")
    (ros::advertise "/aero_controller/command" trajectory_msgs::JointTrajectory)
    (ros::subscribe "/aero_controller/state"
		    pr2_controllers_msgs::JointTrajectoryControllerState
		    #'send self :controller-callback 1)
    (ros::subscribe "/object/left_pose"
		    geometry_msgs::Pose #'send self :tfl-callback 1)
    (ros::subscribe "/object/right_pose"
		    geometry_msgs::Pose #'send self :tfr-callback 1)
    (setf base-coords (make-cascoords))
    )
  (:default-controller-list ()
   (list :larm-controller
         :rarm-controller
         :torso-controller
         :fullbody-controller)
   )
  (:default-controller ()
    (mapcar
     #'(lambda (ctype) (car (send self ctype))) (send self :default-controller-list))
    )
  (:fullbody-controller ()
   (send-message self robot-interface :default-controller)
   )
  (:larm-controller ()
   (list
    (list
     (cons :controller-action "/larm_controller/follow_joint_trajectory_action")
     (cons :controller-state  "/larm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names
	   (list "LARM_SHOULDER_P" "LARM_SHOULDER_R" "LARM_SHOULDER_Y"
		 "LARM_ELBOW" "LARM_WRIST_Y" "LARM_WRIST_P"))
     ))
   )
  (:rarm-controller ()
   (list
    (list
     (cons :controller-action "/rarm_controller/follow_joint_trajectory_action")
     (cons :controller-state  "/rarm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names
	   (list "RARM_SHOULDER_P" "RARM_SHOULDER_R" "RARM_SHOULDER_Y"
		 "RARM_ELBOW" "RARM_WRIST_Y" "RARM_WRIST_P"))
     ))
   )
  (:torso-controller ()
   (list
    (list
     (cons :controller-action "/torso_controller/follow_joint_trajectory_action")
     (cons :controller-state  "/torso_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "WAIST_Y" "WAIST_P" "WAIST_R"))
     ))
   )
  (:angle-vector (av &optional (tm 5000))
    (let ((joint-names (list "r_shoulder_p_joint"
			     "r_shoulder_r_joint"
			     "r_shoulder_y_joint"
			     "r_elbow_joint"
			     "r_wrist_y_joint"
			     "r_wrist_p_joint"
			     "r_wrist_r_joint"
			     ;; "r_thumb_joint"
			     "l_shoulder_p_joint"
			     "l_shoulder_r_joint"
			     "l_shoulder_y_joint"
			     "l_elbow_joint"
			     "l_wrist_y_joint"
			     "l_wrist_p_joint"
			     "l_wrist_r_joint"
			     ;; "l_thumb_joint"
			     "waist_y_joint"
			     "waist_p_joint"
			     "waist_r_joint"
			     "neck_y_joint"
			     "neck_p_joint"
			     "neck_r_joint"))
	  (send-av (float-vector
		    (deg2rad (elt av 0))
		    (deg2rad (elt av 1))
		    (deg2rad (elt av 2))
		    (deg2rad (elt av 3))
		    (deg2rad (elt av 4))
		    (deg2rad (elt av 5))
		    (deg2rad (elt av 6))
		    ;; (deg2rad (elt av 7))
		    (deg2rad (elt av 11))
		    (deg2rad (elt av 12))
		    (deg2rad (elt av 13))
		    (deg2rad (elt av 14))
		    (deg2rad (elt av 15))
		    (deg2rad (elt av 16))
		    (deg2rad (elt av 17))
		    ;; (deg2rad (elt av 18))
		    (deg2rad (elt av 22))
		    (deg2rad (elt av 23))
		    (deg2rad (elt av 24))
		    (deg2rad (elt av 25))
		    (deg2rad (elt av 26))
		    (deg2rad (elt av 27))))
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
     (rad2deg (cdr (assoc "r_shoulder_p_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_shoulder_r_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_shoulder_y_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_elbow_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_wrist_y_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_wrist_p_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_wrist_r_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "r_thumb_joint" av :test #'equal)) )
     0
     0
     0
     (rad2deg (cdr (assoc "l_shoulder_p_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "l_shoulder_r_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "l_shoulder_y_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "l_elbow_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "l_wrist_y_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "l_wrist_p_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "l_wrist_r_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "l_thumb_joint" av :test #'equal)) )
     0
     0
     0
     (rad2deg (cdr (assoc "waist_y_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "waist_p_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "waist_r_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "neck_y_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "neck_p_joint" av :test #'equal)) )
     (rad2deg (cdr (assoc "neck_r_joint" av :test #'equal)) )
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
  (:tfl-callback (msg)
    (let (position
	  orientation
	  )
      (setq position (send msg :position))
      (setq orientation (send msg :orientation))
      (setq object-l (make-coords :pos (float-vector
					(* (send position :x) 1000)
					(* (send position :y) 1000)
					(* (send position :z) 1000))
				  :rot (quaternion2matrix
					(float-vector (send orientation :x)
						      (send orientation :y)
						      (send orientation :z)
						      (send orientation :w)))
				  ))
      ))
  (:tfr-callback (msg)
    (let (position
	  orientation
	  )
      (setq position (send msg :position))
      (setq orientation (send msg :orientation))
      (setq object-r (make-coords :pos (float-vector
					(* (send position :x) 1000)
					(* (send position :y) 1000)
					(* (send position :z) 1000))
				  :rot (quaternion2matrix
					(float-vector (send orientation :x)
						      (send orientation :y)
						      (send orientation :z)
						      (send orientation :w)))
				  ))
      ))
  (:tf (arm)
    (let (object
	  )
      (ros::spin-once)
      (if (eq arm :larm) (setq object object-l) (setq object object-r) )
      (if (and (eps= (elt (send object :pos) 0) 0.0)
	       (eps= (elt (send object :pos) 1) 0.0)
	       (eps= (elt (send object :pos) 2) 0.0))
	  (setq object nil))
      object))

  );; end of defmethod aero-upper-robot



(defun aero-init (&rest args)
  (when (not (boundp '*ri*))
    (let ((disp (copy-object x::*display*))
	  )
      (setf x::*display* 0)
      (setq *ri* (instance* aero-upper-interface :init args))
      (setf x::*display* disp)
      ))
  (if (not (boundp '*aero*))
      (setq *aero* (instance AeroUpperRobot-robot :init))
    )
  ;;(setq ((send *aero* :head :neck-p) . max-angle) 40)this limit has a bug
  (setq ((send *aero* :head :neck-p) . max-angle) 39)
  (setq ((send *aero* :torso :waist-p) . max-angle) 30)
  (setq ((send *aero* :rarm :elbow-p) . max-angle) -15)
  (setq ((send *aero* :larm :elbow-p) . max-angle) -15)
  (setq ((send *aero* :rarm :shoulder-p) . min-angle) -89)
  (setq ((send *aero* :larm :shoulder-p) . min-angle) -89)
  (setq ((send *aero* :rarm :wrist-r) . max-angle) 39)
  (setq ((send *aero* :larm :wrist-r) . max-angle) 39)
  (setq ((send *aero* :rarm :wrist-r) . min-angle) -39)
  (setq ((send *aero* :larm :wrist-r) . min-angle) -39)
  (setq ((send *aero* :torso :waist-y) . min-angle) -90)
  (setq ((send *aero* :torso :waist-y) . max-angle) 90)
  (setq ((send *aero* :head :neck-y) . min-angle) -45)
  (setq ((send *aero* :head :neck-y) . max-angle) 45)
  (send *aero* :angle-vector (send *ri* :actual-vector))
  )

(defun load-controllers ()
  (load "package://aeroeus/aero-wheels.l")
  (load "package://aeroeus/aero-hand.l")
  (load "package://aeroeus/aero-torso.l")
  )

(defvar *base-time* 1000)
(defvar *real* nil)
