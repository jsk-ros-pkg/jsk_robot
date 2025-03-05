(require :franka-common-interface "package://panda_eus/euslisp/franka-common-interface.l")
(require :dual_panda-utils "package://panda_eus/euslisp/dual_panda-utils.l")

(defclass dual_panda-robot-interface
  :super franka-common-interface
  :slots ())

(defmethod dual_panda-robot-interface
  (:init
    (&rest args)
    (let ((all-arms (list :rarm :larm)))
      (send-super* :init :robot dual_panda-robot
                   :joint-states-topic "dual_panda/joint_states"
                   :all-arms all-arms
                   :all-arm-aliases (mapcar #'(lambda (arm) nil) all-arms)
                   :error-topics (mapcar
                                   #'(lambda (arm)
                                       (format nil "/dual_panda/~a/has_error"
                                               (string-downcase (string arm))))
                                   all-arms)
                   :error-topic-types (mapcar #'(lambda (arm) std_msgs::Bool) all-arms)
                   :error-recovery-action "/dual_panda/error_recovery"
                   :gripper-action-prefixes (mapcar
                                              #'(lambda (arm)
                                                  (format nil "/dual_panda/~a"
                                                          (string-downcase (string arm))))
                                              all-arms)
                   args)))
  (:default-controller
    ()
    (list
      (list
        (cons :controller-action "/dual_panda/dual_panda_effort_joint_trajectory_controller/follow_joint_trajectory")
        (cons :controller-state  "/dual_panda/dual_panda_effort_joint_trajectory_controller/state")
        (cons :action-type control_msgs::FollowJointTrajectoryAction)
        (cons :joint-names (send-all (send robot :joint-list) :name))))))

(defun dual_panda-init ()
  (setq *ri* (instance dual_panda-robot-interface :init))
  (setq *robot* (dual_panda)))

(provide :dual_panda-interface)
