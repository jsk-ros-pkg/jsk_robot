(load "package://pr2eus/robot-interface.l")
(require :g1 "package://g1eus/g1.l")

(defclass g1-interface
  :super robot-interface
  :slots (hand-actions-
          hand-joint-names-))

(defmethod g1-interface
  (:arm-to-hand-controller-name
   (arm)
   (case arm
     (:rarm "right_hand_controller")
     (:larm "left_hand_controller")
     (t nil)))

  (:default-hand-joint-names
   (arm)
   (case arm
     (:rarm
      (list
       "R_thumb_proximal_yaw_joint"
       "R_thumb_proximal_pitch_joint"
       "R_index_proximal_joint"
       "R_middle_proximal_joint"
       "R_ring_proximal_joint"
       "R_pinky_proximal_joint"))
     (:larm
      (list
       "L_thumb_proximal_yaw_joint"
       "L_thumb_proximal_pitch_joint"
       "L_index_proximal_joint"
       "L_middle_proximal_joint"
       "L_ring_proximal_joint"
       "L_pinky_proximal_joint"))
     (t nil)))

  (:upper-body-controller
   ()
   (list
    (list
     (cons :controller-action "upper_body_controller/follow_joint_trajectory")
     (cons :controller-state "upper_body_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names    (list
			    "waist_yaw_joint"
			    "waist_roll_joint"
			    "waist_pitch_joint"
			    "left_shoulder_pitch_joint"
			    "left_shoulder_roll_joint"
			    "left_shoulder_yaw_joint"
			    "left_elbow_joint"
			    "left_wrist_roll_joint"
			    "left_wrist_pitch_joint"
			    "left_wrist_yaw_joint"
			    "right_shoulder_pitch_joint"
			    "right_shoulder_roll_joint"
			    "right_shoulder_yaw_joint"
			    "right_elbow_joint"
			    "right_wrist_roll_joint"
			    "right_wrist_pitch_joint"
			    "right_wrist_yaw_joint")))))

  (:default-controller
   ()
   (append
    (send self :upper-body-controller)
    )
   )

  (:init (&rest args)
    (send-super* :init :robot g1-robot args)
    (setq hand-actions- (make-hash-table))
    (setq hand-joint-names- (make-hash-table))
    (dolist (arm (list :rarm :larm))
      (let* ((controller-name (send self :arm-to-hand-controller-name arm))
             (action-name (format nil "~a/follow_joint_trajectory" controller-name))
             (joint-names (ros::get-param (format nil "~a/joints" controller-name)
                                           (send self :default-hand-joint-names arm)))
             action)
        (sethash arm hand-joint-names- joint-names)
        (setq action
              (instance ros::simple-action-client :init
                        action-name
                        control_msgs::FollowJointTrajectoryAction
                        :groupname groupname))
        (if (and joint-action-enable (send action :wait-for-server 3))
          (sethash arm hand-actions- action)
          (ros::ros-warn "~A is not respond" action)))))

;;  (:larm (&rest args)
;;   (send* self :limb :larm args))

;;  (:rarm (&rest args)
;;   (send* self :limb :rarm args))

;;  (:limb
;;   (arm &rest args)
;;   (case (car args)
;;     (:hand-angle-vector
;;      (send* self :hand-angle-vector arm (cdr args)))
;;     (:cancel-hand-angle-vector
;;      (send self :cancel-hand-angle-vector arm))
;;     (:hand-interpolatingp
;;      (send self :hand-interpolatingp arm))
;;     (t
;;      (send* robot arm args))))

  (:stop-grasp
   (arm)
   (send self :hand-angle-vector arm #f(0 0 0 0 0 0) 1000))

  (:prepare-grasp
   (arm)
   (send self :hand-angle-vector arm #f(100 0 0 0 0 0) 1000))
  
  (:start-grasp
   (arm)
   (send self :hand-angle-vector arm #f(40 20 80 80 80 80) 1000))

  (:default-grasp
   (arm)
   (send self :hand-angle-vector arm #f(10 30 100 100 100 100) 1000))

  (:hand-angle-vector
   (arm av tm &key (wait t))
   (let ((action (gethash arm hand-actions-))
         (names (gethash arm hand-joint-names-)))
     (cond
       ((null action)
        (ros::ros-warn "hand action for ~A is not available" arm)
        nil)
       ((null names)
        (ros::ros-warn "joint names for ~A hand are not available" arm)
        nil)
       ((/= (length av) (length names))
        (ros::ros-warn "hand angle-vector length ~A does not match joint names length ~A for ~A"
                       (length av) (length names) arm)
        nil)
       (t
        (let ((av-list (coerce av cons))
              res
              (goal (instance control_msgs::FollowJointTrajectoryActionGoal :init))
              (traj (instance trajectory_msgs::JointTrajectory :init)))
          (send traj :header :seq 1)
          (send traj :header :stamp (ros::time-now))
          (send traj :joint_names names)
          (send traj :points
                (list (instance trajectory_msgs::JointTrajectoryPoint
                                :init
                                :positions (mapcar #'deg2rad av-list)
                                :time_from_start (ros::time (/ tm 1000.0)))))
          (send goal :goal :trajectory traj)
          (send action :send-goal goal)
          (setq res (send action :get-result))
          (when wait
            (send action :wait-for-result)
            (setq res (send action :get-result)))
          (unless (eq (send res :error_code) 0)
            (warning-message 3 ":hand-angle-vector error code ~A returns from JTA: ~A~%"
                             (send res :error_code) (send res :error_string)))
          res)))))

  (:cancel-hand-angle-vector
   (arm)
   (let ((action (gethash arm hand-actions-)))
     (if action
       (send action :cancel-goal)
       (ros::ros-warn "hand action for ~A is not available" arm))))

  (:hand-interpolatingp
   (arm)
   (send self :spin-once)
   (let ((action (gethash arm hand-actions-)))
     (and action
          (eq (send action :get-state) ros::*simple-goal-state-active*)))))

(defun g1-init ()
  (unless (boundp '*ri*)
    (setq *ri* (instance g1-interface :init)))
  (unless (boundp '*g1*)
    (setq *g1* (instance g1-robot :init))))
