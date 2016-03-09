(ros::roseus "fetch")

(require "package://fetcheus/fetch.l")
(require "package://pr2eus/pr2-interface.l")

(defclass fetch-interface
  :super robot-move-base-interface
  :slots ()
  )

(defmethod fetch-interface
  (:init (&rest args)
   (prog1 (send-super* :init :robot fetch-robot :base-frame-id "/base_link" :odom-topic "/odom" args)
     (send self :add-controller :arm-controller)
     (send self :add-controller :torso-controller)
     (send self :add-controller :head-controller)
     ))
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
  ;;
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

#|
(setq *ri* (instance fetch-interface :init))
(setq *fetch* (fetch))
(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
|#
