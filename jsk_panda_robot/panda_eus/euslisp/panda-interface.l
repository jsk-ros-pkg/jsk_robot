(require :robot-interface "package://pr2eus/robot-interface.l")
(require :franka "package://panda_eus/models/panda.l")

(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")

(defclass panda-robot-interface
  :super robot-interface
  :slots (gripper-action
          error-recovery-act error
          gripper-grasp-action gripper-move-action gripper-homing-action gripper-stop-action
          )
  )
(defmethod panda-robot-interface
  (:init
   (&rest args &key ((:controller-timeout ct) nil))
   (prog1
       (send-super* :init :robot panda-robot
                    :joint-states-topic "joint_states"
                    :controller-timeout ct args)
     ;; for error recovery
     (ros::create-nodehandle "error_group")
     (ros::subscribe "/panda/arm/has_error" std_msgs::Bool
                     #'send self :callback-arm-error 1 :groupname "error_group")
     (setq error-recovery-act (instance ros::simple-action-client :init
                                        "/franka_control/error_recovery"
                                        franka_msgs::ErrorRecoveryAction
                                        :groupname "error_group"
                                        ))
     ;; actions for gripper
     (setq gripper-grasp-action
       (instance ros::simple-action-client :init
             "/franka_gripper/grasp"
             franka_gripper::GraspAction))
     (setq gripper-homing-action
       (instance ros::simple-action-client :init
             "/franka_gripper/homing"
             franka_gripper::HomingAction))
     (setq gripper-move-action
       (instance ros::simple-action-client :init
             "/franka_gripper/move"
             franka_gripper::MoveAction))
     (setq gripper-stop-action
       (instance ros::simple-action-client :init
             "/franka_gripper/stop"
             franka_gripper::StopAction))
     ))
  (:default-controller
   ()
   (list
    (list
     (cons :controller-action "/position_joint_trajectory_controller/follow_joint_trajectory")
     (cons :controller-state  "/position_joint_trajectory_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (send-all (send robot :joint-list) :name))
     )))
  (:rarm-controller
   ()
   (list
    (list
     (cons :controller-action "/rarm_controller/follow_joint_trajectory")
     (cons :controller-state  "/position_joint_trajectory_controller/follow_joint_trajectory")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names
           (remove-if #'(lambda (jn) (substringp "finger" jn))
                      (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n))
                              (send-all (send robot :rarm :joint-list) :name)))))))
  (:hand-controller
   ()
   (list
    (list
     (cons :controller-action "/hand_controller/follow_joint_trajectory")
     (cons :controller-state "/hand_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "left_finger_joint1" "left_finger_joint2") ))))
  (:set-joint-pd-gain
   (joint-name pgain dgain)
   (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
     (send req :config :doubles
           (list (instance dynamic_reconfigure::DoubleParameter :init
                           :name "p" :value pgain)
                 (instance dynamic_reconfigure::DoubleParameter :init
                           :name "d" :value dgain)))
     (ros::service-call
      (format nil "/position_joint_trajectory_controller/gains/~A/set_parameters" joint-name)
      req)
     ))
  (:set-all-joint-pd-gain
   (pgain dgain)
   (dolist (j (send robot :joint-list))
     (send self :set-joint-pd-gain (send j :name) pgain dgain))
   )
  (:check-error ()
                (ros::spin-once "error_group")
                (or error)
                )
  (:callback-arm-error (msg)
                        (setq error (send msg :data))
                        )
  (:wait-recover-error () (send error-recovery-act :wait-for-result))
  (:recover-error (&key (wait t))
                  (let ((goal (instance franka_msgs::ErrorRecoveryActionGoal :init)))
                    (send goal :header :stamp (ros::time-now))
                    (send error-recovery-act :send-goal goal)
                    (if wait (send self :wait-recover-error))
                    ))
  ;; gripper action for real-controller
  (:send-gripper-grasp-action
   (act width speed force &key (wait t) (inner 0.005) (outer 0.005))
   (let ((goal (instance franka_gripper::GraspActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     (send goal :goal :width width) ;; [m]
     (send goal :goal :speed speed) ;; [m/s]
     (send goal :goal :force force) ;; [N]
     (send goal :goal :epsilon :inner inner) ;; [m]
     (send goal :goal :epsilon :outer outer) ;; [m]
     ;;
     (send act :send-goal goal)
     (if wait (send act :wait-for-result))
     ))
  (:send-gripper-homing-action
   (act &key (wait t))
   (let ((goal (instance franka_gripper::HomingActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     ;;
     (send act :send-goal goal)
     (if wait (send act :wait-for-result))
     ))
  (:send-gripper-move-action
   (act width speed &key (wait t))
   (let ((goal (instance franka_gripper::MoveActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     (send goal :goal :width width) ;; [m]
     (send goal :goal :speed speed) ;; [m/s]
     ;;
     (send act :send-goal goal)
     (if wait (send act :wait-for-result))
     ))
  (:send-gripper-stop-action
   (act &key (wait t))
   (let ((goal (instance franka_gripper::StopActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     ;;
     (send act :send-goal goal)
     (if wait (send act :wait-for-result))
     ))
  (:stop-gripper
   (arm &key (wait nil))
   (let (acts)
     (case
         arm
       (:arm (setq acts (list gripper-stop-action)))
       )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-stop-action act
               pos (/ (* 1000 0.08) tm) :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  (:homing-gripper
   (arm &key (wait nil))
   (let (acts)
     (case
         arm
       (:arm (setq acts (list gripper-homing-action)))
       )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-homing-action act :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  (:start-grasp
   (arm &optional (pos 0.0) &key (effort 10.0) (tm 500) (wait nil)) ;; TODO :arms is not implemented
   (let (acts)
     (case
         arm
       (:arm (setq acts (list gripper-grasp-action)))
       )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-grasp-action act
               pos (/ (* 1000 0.08) tm) effort :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  (:stop-grasp
   (arm &key (wait nil)) ;; TODO :arms is not implemented
   (unless (memq arm '(:arm :arms))
     (error "you must specify arm ~A from ~A" (car args) '(:arm :arms))
     (return-from :stop-grasp nil))
   (send self :move-gripper arm 0.08 :tm 500 :wait wait)
   )
  (:move-gripper
   (arm pos &key (tm 500) (timeout 5000) (wait nil))  ;; TODO :arms is not implemented
   (let (acts)
     (case
         arm
       (:arm (setq acts (list gripper-move-action)))
       )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-move-action act
               pos (/ (* 1000 0.08) tm) :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  )

;; grasp controller ...
(defun panda-init ()
  (setq *ri* (instance panda-robot-interface :init))
  (setq *robot* (panda))
  )

#|
(send *ri* :set-all-joint-pd-gain 1000.0 5.0) ;; default
(send *ri* :set-all-joint-pd-gain  300.0 5.0) ;; hard
(send *ri* :set-all-joint-pd-gain   30.0 0.5) ;; soft
|#
