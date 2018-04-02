(require :aero "package://aeroeus/aero.l")
(require :robot-interface "package://pr2eus/robot-interface.l")

(defclass aero-interface
  :super robot-interface
  :slots ()
  )
(defmethod aero-interface
  (:init
   (&rest args)
   (send-super* :init :robot aeroupperrobot-robot
                :groupname "aero_interface" args)
   (dolist (ct (list :rarm-controller
                     :larm-controller
                     :head-controller
                     :waist-controller
                     :lift-controller))
     (send self :add-controller ct :create-actions nil))
   )
  ;;
  (:default-controller nil
   (append (send self :rarm-controller)
           (send self :larm-controller)
           (send self :head-controller)
           (send self :waist-controller)
           (send self :lift-controller)))
  (:rarm-controller nil
   (list
    (list
     (cons :controller-action "rarm_controller/follow_joint_trajectory")
     (cons :controller-state  "rarm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "r_shoulder_p_joint" "r_shoulder_r_joint" "r_shoulder_y_joint"
                              "r_elbow_joint" "r_wrist_y_joint" "r_wrist_p_joint" "r_wrist_r_joint"
                              "r_hand_y_joint"
                              ))
     ))
   )
  (:larm-controller nil
   (list
    (list
     (cons :controller-action "larm_controller/follow_joint_trajectory")
     (cons :controller-state  "larm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "l_shoulder_p_joint" "l_shoulder_r_joint" "l_shoulder_y_joint"
                              "l_elbow_joint" "l_wrist_y_joint" "l_wrist_p_joint" "l_wrist_r_joint"
                              "l_hand_y_joint"
                              ))
     ))
   )
  (:head-controller nil
   (list
    (list
     (cons :controller-action "head_controller/follow_joint_trajectory")
     (cons :controller-state  "head_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "neck_y_joint" "neck_p_joint" "neck_r_joint"))
     ))
   )
  (:waist-controller nil
   (list
    (list
     (cons :controller-action "waist_controller/follow_joint_trajectory")
     (cons :controller-state  "waist_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "waist_y_joint" "waist_p_joint" "waist_r_joint"))
     ))
   )
  (:lift-controller nil
   (list
    (list
     (cons :controller-action "lifter_controller/follow_joint_trajectory")
     (cons :controller-state  "lifter_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "ankle_joint" "knee_joint"))
     ))
   )
  )

(defun aero-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* aero-interface :init args)))
  (if (not (boundp '*robot*))
      (setq *robot* (aeroupperrobot)))
  )

#|
(aero-init)
(send *ri* :state)
(send *robot* :angle-vector (send *ri* :state :reference-vector))
(send *robot* :angle-vector (send *ri* :state :potentio-vector))

(send *ri* :angle-vector (send *robot* :angle-vector) 1000)

;; should add controllers
(send *ri* :angle-vector (send *robot* :angle-vector) 1000 :larm-controller)
(send *ri* :angle-vector (send *robot* :angle-vector) 1000 :rarm-controller)
(send *ri* :angle-vector (send *robot* :angle-vector) 1000 :waist-controller)
(send *ri* :angle-vector (send *robot* :angle-vector) 1000 :lift-controller)
(send *ri* :angle-vector (send *robot* :angle-vector) 1000 :head-controller)

(send *robot* :angle-vector (send *ri* :state :potentio-vector))

(send *robot* :larm :elbow-p :joint-angle -30)
(send *ri* :angle-vector (send *robot* :angle-vector) 4000)

(send *robot* :torso :lifter-ankle :joint-angle 30)
(send *robot* :torso :lifter-knee  :joint-angle -30)
(send *ri* :angle-vector (send *robot* :angle-vector) 4000)

(send *robot* :torso :lifter-ankle :joint-angle 45)
(send *robot* :torso :lifter-knee  :joint-angle -45)
(send *ri* :angle-vector (send *robot* :angle-vector) 4000)

(dolist (j (send *robot* :joint-list))
  (format t "\"~A\" ~5,5F ~5,5F~%"
          (send j :name)
          (deg2rad
           (+ (* 0.75 (send j :max-angle))
              (* 0.25 (send j :min-angle))))
          (deg2rad
           (+ (* 0.25 (send j :max-angle))
              (* 0.75 (send j :min-angle))))
          ))
|#
