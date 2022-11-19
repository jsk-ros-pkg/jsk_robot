(require :robot-interface "package://pr2eus/robot-interface.l")
(require :dual_panda "package://panda_eus/models/dual_panda.l")

(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")

(defclass dual_panda-robot-interface
  :super robot-interface
  :slots (error-recovery-act
          r-error l-error
          r-gripper-grasp-action r-gripper-move-action r-gripper-homing-action r-gripper-stop-action
          l-gripper-grasp-action l-gripper-move-action l-gripper-homing-action l-gripper-stop-action
          )
  )
(defmethod dual_panda-robot-interface
  (:init
   (&rest args)
   (prog1
       (send-super* :init :robot dual_panda-robot
                    :joint-states-topic "dual_panda/joint_states"
                    args)
     ;; for error recovery
     (ros::create-nodehandle "error_group")
     (ros::subscribe "/dual_panda/rarm/has_error" std_msgs::Bool
                     #'send self :callback-rarm-error 1 :groupname "error_group")
     (ros::subscribe "/dual_panda/larm/has_error" std_msgs::Bool
                     #'send self :callback-larm-error 1 :groupname "error_group")
     (setq error-recovery-act (instance ros::simple-action-client :init
                                        "/dual_panda/error_recovery"
                                        franka_msgs::ErrorRecoveryAction
                                        :groupname "error_group"
                                        ))
     ;; actions for gripper
     (setq r-gripper-grasp-action
           (instance ros::simple-action-client :init
                     "/dual_panda/rarm/franka_gripper/grasp"
                     franka_gripper::GraspAction))
     (setq r-gripper-homing-action
           (instance ros::simple-action-client :init
                     "/dual_panda/rarm/franka_gripper/homing"
                     franka_gripper::HomingAction))
     (setq r-gripper-move-action
           (instance ros::simple-action-client :init
                     "/dual_panda/rarm/franka_gripper/move"
                     franka_gripper::MoveAction))
     (setq r-gripper-stop-action
           (instance ros::simple-action-client :init
                     "/dual_panda/rarm/franka_gripper/stop"
                     franka_gripper::StopAction))

     (setq l-gripper-grasp-action
           (instance ros::simple-action-client :init
                     "/dual_panda/larm/franka_gripper/grasp"
                     franka_gripper::GraspAction))
     (setq l-gripper-homing-action
           (instance ros::simple-action-client :init
                     "/dual_panda/larm/franka_gripper/homing"
                     franka_gripper::HomingAction))
     (setq l-gripper-move-action
           (instance ros::simple-action-client :init
                     "/dual_panda/larm/franka_gripper/move"
                     franka_gripper::MoveAction))
     (setq l-gripper-stop-action
           (instance ros::simple-action-client :init
                     "/dual_panda/larm/franka_gripper/stop"
                     franka_gripper::StopAction))
     ))
  (:default-controller
   ()
   (list
    (list
     (cons :controller-action "/dual_panda/dual_panda_effort_joint_trajectory_controller/follow_joint_trajectory")
     (cons :controller-state  "/dual_panda/dual_panda_effort_joint_trajectory_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (send-all (send robot :joint-list) :name))
     )))
  (:set-joint-pd-gain
   (joint-name pgain dgain)
   "Set P gain and D gain of each joint"
   (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
     (send req :config :doubles
           (list (instance dynamic_reconfigure::DoubleParameter :init
                           :name "p" :value pgain)
                 (instance dynamic_reconfigure::DoubleParameter :init
                           :name "d" :value dgain)))
     (ros::service-call
      (format nil "/dual_panda/dual_panda_effort_joint_trajectory_controller/gains/~A/set_parameters" joint-name)
      req)
     ))
  (:set-all-joint-pd-gain
   (pgain dgain)
   "Set P gain and D gain of all joints"
   (dolist (j (send robot :joint-list))
     (send self :set-joint-pd-gain (send j :name) pgain dgain))
   )
  (:check-error
   ()
   "Check if the robot has error.
If this method returns T, you must call :recover-error to move the robot.
"
   (ros::spin-once "error_group")
   (or r-error l-error)
   )
  (:callback-rarm-error
   (msg)
   (setq r-error (send msg :data))
   )
  (:callback-larm-error
   (msg)
   (setq l-error (send msg :data))
   )
  (:wait-recover-error () (send error-recovery-act :wait-for-result))
  (:recover-error
   (&key (wait t))
   "Recover from errors and reflexes.
Details: `ErrorRecoveryAction` part of https://frankaemika.github.io/docs/franka_ros.html#franka-control
"
   (let ((goal (instance franka_msgs::ErrorRecoveryActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     (send error-recovery-act :send-goal goal)
     (if wait (send self :wait-recover-error))
     ))
  ;; gripper action for real-controller
  (:send-gripper-grasp-action
   (act width speed force &key (wait t) (inner 0.005) (outer 0.07))
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
   "Abort a running gripper action. This can be used to stop applying forces after grasping.
Details: `StopAction` part of https://frankaemika.github.io/docs/franka_ros.html#franka-gripper
"
   (let (acts)
     (case
      arm
      (:rarm (setq acts (list r-gripper-stop-action)))
      (:larm (setq acts (list l-gripper-stop-action)))
      (:arms (setq acts (list r-gripper-stop-action l-gripper-stop-action)))
      )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-stop-action act :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  (:homing-gripper
   (arm &key (wait nil))
   "Home the gripper and update the maximum width given the mounted fingers (i.e., calibrate & initialize the gripper).
Details: `HomingAction` part of https://frankaemika.github.io/docs/franka_ros.html#franka-gripper
"
   (let (acts)
     (case
      arm
      (:rarm (setq acts (list r-gripper-homing-action)))
      (:larm (setq acts (list l-gripper-homing-action)))
      (:arms (setq acts (list r-gripper-homing-action l-gripper-homing-action)))
      )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-homing-action act :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  (:start-grasp
   (arm &key (width 0.0) (effort 80.0) (tm 500) (wait nil) (inner 0.005) (outer 0.06))
   "Try to grasp at the desired `width` with the desired `effort` while closing with the desired speed calculated from `tm`.
Arguments:
- arm : :rarm, :larm, or :arms
- width : target distance between the fingers [m]
- effort : target effort [N]
- tm : time to target [ms]. This will be converted to the movement speed
- wait : if this argument is T, this method waits until the movement finishes
- inner : lower admissible error of width. If this is violated, the gripper stops applying forces
- outer : upper admissible error of width. If this is violated, the gripper stops applying forces
          Details: https://github.com/ykawamura96/jsk_robot/pull/1#issuecomment-860324988
Details: `GraspAction` part of https://frankaemika.github.io/docs/franka_ros.html#franka-gripper
"
   (let (acts)
     (case
      arm
      (:rarm (setq acts (list r-gripper-grasp-action)))
      (:larm (setq acts (list l-gripper-grasp-action)))
      (:arms (setq acts (list r-gripper-grasp-action l-gripper-grasp-action)))
      )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-grasp-action act
               width (/ (* 1000 0.08) tm) effort :wait nil :inner inner :outer outer))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  (:stop-grasp
   (arm &key (wait nil) (width 0.08))
   "Open the gripper to the target `width` [m]"
   (unless (memq arm '(:larm :rarm :arms))
     (error "you must specify arm ~A from ~A" (car args) '(:larm :rarm :arms))
     (return-from :stop-grasp nil))
   (send self :move-gripper arm width :tm 500 :wait wait)
   )
  (:move-gripper
   (arm width &key (tm 500) (wait nil))
   "Move the gripper to the target `width` [m] while closing with the desired speed calculated from `tm` [ms].
Details: `MoveAction` part of https://frankaemika.github.io/docs/franka_ros.html#franka-gripper
"
   (let (acts)
     (case
      arm
      (:rarm (setq acts (list r-gripper-move-action)))
      (:larm (setq acts (list l-gripper-move-action)))
      (:arms (setq acts (list r-gripper-move-action l-gripper-move-action)))
      )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-move-action act
               width (/ (* 1000 0.08) tm) :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  )

(defun dual_panda-init ()
  (setq *ri* (instance dual_panda-robot-interface :init))
  (setq *robot* (dual_panda))
  )

#|
(send *ri* :set-all-joint-pd-gain 1000.0 5.0) ;; default
(send *ri* :set-all-joint-pd-gain  300.0 5.0) ;; hard
(send *ri* :set-all-joint-pd-gain   30.0 0.5) ;; soft
|#
