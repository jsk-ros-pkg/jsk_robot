(load "package://pr2eus/robot-interface.l")
(require :baxter "package://baxtereus//baxter.l")
(ros::load-ros-manifest "control_msgs")

(defclass baxter-interface
  :super robot-interface
  :slots ())
(defmethod baxter-interface
  (:init (&rest args)
   (prog1 (send-super :init :robot baxter-robot :joint-states-topic "/robot/joint_states")
     (send self :add-controller :larm-controller)
     (send self :add-controller :rarm-controller)
     ))
  (:default-controller ()
   (append
    (send self :larm-controller)
    (send self :rarm-controller)))
  (:larm-controller ()
   (list
    (list
     (cons :controller-action "/robot/limb/left/follow_joint_trajectory")
     (cons :controller-state "/robot/limb/left/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "left_s0" "left_s1" "left_e0" "left_e1" "left_w0" "left_w1")))))
  (:rarm-controller ()
   (list
    (list
     (cons :controller-action "/robot/limb/right/follow_joint_trajectory")
     (cons :controller-state "/robot/limb/right/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "right_s0" "right_s1" "right_e0" "right_e1" "right_w0" "right_w1")))))
  )

(defun baxter-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance baxter-interface :init)))
  (if (not (boundp '*baxter*))
      (setq *baxter* (instance baxter-robot :init)))
  )
