(require :franka-common-interface "package://panda_eus/euslisp/franka-common-interface.l")
(require :panda-utils "package://panda_eus/euslisp/panda-utils.l")

(defclass panda-robot-interface
  :super franka-common-interface
  :slots ())

(defmethod panda-robot-interface
  (:init
    (&rest args &key (robot panda-robot))
    (send-super* :init :robot robot
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

(defun panda-init ()
  (setq *ri* (instance panda-robot-interface :init))
  (setq *robot* (panda)))

(provide :panda-interface)
