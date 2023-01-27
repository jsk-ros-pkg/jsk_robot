(require :robot-interface "package://pr2eus/robot-interface.l")
(require :dual_panda "package://panda_eus/models/dual_panda.l")

(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")

(defclass dual_panda-robot-interface
  :super robot-interface
  :slots (error-recovery-act
          r-error l-error
          gripper-grasp-actions gripper-move-actions gripper-homing-actions gripper-stop-actions
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
     (setq gripper-grasp-actions (make-hash-table))
     (setq gripper-homing-actions (make-hash-table))
     (setq gripper-move-actions (make-hash-table))
     (setq gripper-stop-actions (make-hash-table))
     (dolist (arm (list :rarm :larm))
       (sethash arm gripper-grasp-actions
                (instance ros::simple-action-client :init
                          (format nil "/dual_panda/~a/franka_gripper/grasp"
                                  (string-downcase (string arm)))
                          franka_gripper::GraspAction))
       (sethash arm gripper-homing-actions
                (instance ros::simple-action-client :init
                          (format nil "/dual_panda/~a/franka_gripper/homing"
                                  (string-downcase (string arm)))
                          franka_gripper::HomingAction))
       (sethash arm gripper-move-actions
                (instance ros::simple-action-client :init
                          (format nil "/dual_panda/~a/franka_gripper/move"
                                  (string-downcase (string arm)))
                          franka_gripper::MoveAction))
       (sethash arm gripper-stop-actions
                (instance ros::simple-action-client :init
                          (format nil "/dual_panda/~a/franka_gripper/stop"
                                  (string-downcase (string arm)))
                          franka_gripper::StopAction)))
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
   (arm width speed force &key (wait t) (inner 0.005) (outer 0.07))
   (let ((goal (instance franka_gripper::GraspActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     (send goal :goal :width width) ;; [m]
     (send goal :goal :speed speed) ;; [m/s]
     (send goal :goal :force force) ;; [N]
     (send goal :goal :epsilon :inner inner) ;; [m]
     (send goal :goal :epsilon :outer outer) ;; [m]
     ;;
     (send (gethash arm gripper-grasp-actions) :send-goal goal)
     (if wait (send (gethash arm gripper-grasp-actions) :wait-for-result))
     ))
  (:send-gripper-homing-action
   (arm &key (wait t))
   (let ((goal (instance franka_gripper::HomingActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     ;;
     (send (gethash arm gripper-homing-actions) :send-goal goal)
     (if wait (send (gethash arm gripper-homing-actions) :wait-for-result))
     ))
  (:send-gripper-move-action
   (arm width speed &key (wait t))
   (let ((goal (instance franka_gripper::MoveActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     (send goal :goal :width width) ;; [m]
     (send goal :goal :speed speed) ;; [m/s]
     ;;
     (send (gethash arm gripper-move-actions) :send-goal goal)
     (if wait (send (gethash arm gripper-move-actions) :wait-for-result))
     ))
  (:send-gripper-stop-action
   (arm &key (wait t))
   (let ((goal (instance franka_gripper::StopActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     ;;
     (send (gethash arm gripper-stop-actions) :send-goal goal)
     (if wait (send (gethash arm gripper-stop-actions) :wait-for-result))
     ))
  (:arm2arms
   (arm)
   (case arm
     ((:rarm :larm) (list arm))
     (:arms (list :rarm :larm))
     ))
  (:gripper-method-helper
   (action-sender actions arm wait)
   (let ((arms (send self :arm2arms arm)))
     (when arms
       (dolist (a arms)
         (send self action-sender a :wait nil))
       (if wait
         (mapcar #'(lambda (a) (send (gethash a actions) :wait-for-result))
                 arms))
       )))
  (:stop-gripper
   (arm &key (wait nil))
   "Abort a running gripper action. This can be used to stop applying forces after grasping.
Details: `StopAction` part of https://frankaemika.github.io/docs/franka_ros.html#franka-gripper
"
   (send self :gripper-method-helper
         :send-gripper-stop-action gripper-stop-actions arm wait)
   )
  (:homing-gripper
   (arm &key (wait nil))
   "Home the gripper and update the maximum width given the mounted fingers (i.e., calibrate & initialize the gripper).
Details: `HomingAction` part of https://frankaemika.github.io/docs/franka_ros.html#franka-gripper
"
   (send self :gripper-method-helper
         :send-gripper-homing-action gripper-homing-actions arm wait)
   )
  (:gripper
   (&rest args)
   "Get information of gripper
Arguments:
- arm : :rarm, :larm, or :arms
- type : :position ([m])
Example: (send self :gripper :rarm :position) => 0.00
"
   (when (eq (car args) :arms)
     (return-from :gripper
                  (mapcar #'(lambda (x)
                              (send self :gripper x (cadr args)))
                          '(:larm :rarm))))
   (unless (memq (car args) '(:larm :rarm))
     (error "you must specify arm ~A from ~A" (car args) '(:larm :rarm))
     (return-from :gripper nil))
   (send self :update-robot-state)
   (case (cadr args)
     (:position (* 0.001 (v. (send robot (car args) :gripper :angle-vector)
                             #f(0 0 0 0 1 0 0 1))))
     ))
  (:gripper-method-with-width-helper
   (action-sender actions arm wait width tm &rest args)
   (let ((arms (send self :arm2arms arm)))
     (when arms
       (dolist (a arms)
         (send* self action-sender a width
                (/ (abs (- (send self :gripper a :position) width))
                   (/ tm 1000.0))
                (append args (list :wait nil))))
       (if wait
         (mapcar #'(lambda (a) (send (gethash a actions) :wait-for-result))
                 arms))
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
   (send self :gripper-method-with-width-helper
         :send-gripper-grasp-action gripper-grasp-actions arm wait width tm
         effort :inner inner :outer outer)
   )
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
   (send self :gripper-method-with-width-helper
         :send-gripper-move-action gripper-move-actions arm wait width tm)
   )
  )

(defun dual_panda-init ()
  (setq *ri* (instance dual_panda-robot-interface :init))
  (setq *robot* (dual_panda))
  )

#|
You can check current gains with rqt_reconfigure.
Be careful changing them with :set-joint-pd-gain or :set-all-joint-pd-gain
|#
