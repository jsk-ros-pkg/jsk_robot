(require :franka-common-interface "package://panda_eus/euslisp/franka-common-interface.l")
(require :fr3-utils "package://panda_eus/euslisp/fr3-utils.l")

(defclass fr3-robot-interface
  :super franka-common-interface
  :slots ())

(defmethod fr3-robot-interface
  (:init
    (&rest args)
    (send-super* :init :robot fr3-robot
                 :joint-states-topic "joint_states"
                 :all-arms (list :rarm)
                 :all-arm-aliases (list :arm)
                 :error-topics (list "/franka_state_controller/franka_states")
                 :error-topic-types (list franka_msgs::FrankaState)
                 :error-recovery-action "/franka_control/error_recovery"
                 :gripper-action-prefixes (list "")
                 args))
  (:default-controller
    ()
    (list
      (list
       (cons :controller-action "/position_joint_trajectory_controller/follow_joint_trajectory")
       (cons :controller-state  "/position_joint_trajectory_controller/state")
       (cons :action-type control_msgs::FollowJointTrajectoryAction)
       (cons :joint-names (send-all (send robot :joint-list) :name)))))
  )

(defun fr3-init ()
  (setq *ri* (instance fr3-robot-interface :init))
  (setq *robot* (fr3)))

(provide :fr3-interface)
