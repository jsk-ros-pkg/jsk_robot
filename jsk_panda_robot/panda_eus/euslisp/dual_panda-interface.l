(require :robot-interface "package://pr2eus/robot-interface.l")
(require :dual_franka "package://panda_eus/models/dual_panda.l")

(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")

(defclass dual_panda-robot-interface
  :super robot-interface
  :slots (gripper-action l-gripper-action gazebop
          error-recovery-act r-error l-error
          r-gripper-grasp-action r-gripper-move-action r-gripper-homing-action r-gripper-stop-action
          l-gripper-grasp-action l-gripper-move-action l-gripper-homing-action l-gripper-stop-action
          )
  )
(defmethod dual_panda-robot-interface
  (:init
   (&rest args &key ((:controller-timeout ct) nil)  &allow-other-keys)
   (prog1
       (send-super* :init :robot dual_panda-robot
                    :joint-states-topic "dual_panda/joint_states"
                    :controller-timeout ct args)
     ;; check whather the environement is Gazebo
     (setq gazebop (numberp (ros::get-param "/gazebo/time_step")))
     (if gazebop (ros::ros-warn "Using Gazebo environment"))

     ;; Controller setting for Gazebo environment
     (when gazebop
       (ros::subscribe "/rhand_controller/state" control_msgs::JointTrajectoryControllerState
                       #'send self :gripper-state-callback :rarm :groupname groupname)
       (ros::subscribe "/lhand_controller/state" control_msgs::JointTrajectoryControllerState
                       #'send self :gripper-state-callback :larm :groupname groupname)
       (setq r-gripper-action (instance ros::simple-action-client :init
                                        "/rhand_controller/follow_joint_trajectory"
                                        control_msgs::FollowJointTrajectoryAction))
       (setq l-gripper-action (instance ros::simple-action-client :init
                                        "/lhand_controller/follow_joint_trajectory"
                                        control_msgs::FollowJointTrajectoryAction))
       (dolist (action (list r-gripper-action l-gripper-action))
         (unless (and joint-action-enable (send action :wait-for-server 3))
           (setq joint-action-enable nil)
           (ros::ros-warn "~A is not respond" action)
           (return)))
       ) ;; when gazebo
     (unless gazebop
       (ros::ros-warn "real franka environemt interface is not impelemented")) ;; TODO

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
  #|
  (:default-controller ;; for gazebo
   ()
   (list
    (car (send self :larm-controller))
    ;; (car (send self :rarm-controller))
    ;; (car (send self :head-controller))
    ))
  |#
  (:larm-controller
   ()
   (list
    (list
     (cons :controller-action "rarm_controller/follow_joint_trajectory")
     (cons :controller-state  "/dual_panda/dual_panda_effort_joint_trajectory_controller/follow_joint_trajectory")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names
           (remove-if #'(lambda (jn) (substringp "finger" jn))
                      (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n))
                              (send-all (send robot :larm :joint-list) :name)))))))
  (:rarm-controller
   ()
   (list
    (list
     (cons :controller-action "rarm_controller/follow_joint_trajectory")
     (cons :controller-state "rarm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names
           (remove-if #'(lambda (jn) (substringp "finger" jn))
                      (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n))
                              (send-all (send robot :rarm :joint-list) :name)))))))
  (:head-controller
   ()
   (list
    (list
     (cons :controller-action "head_controller/follow_joint_trajectory")
     (cons :controller-state "head_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n))
                                (send-all (send robot :head :joint-list) :name))))))
  (:lhand-controller
   ()
   (list
    (list
     (cons :controller-action "lhand_controller/follow_joint_trajectory")
     (cons :controller-state "lhand_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "left_finger_joint1" "left_finger_joint2") ))))
  (:rhand-controller
   ()
   (list
    (list
     (cons :controller-action "rhand_controller/follow_joint_trajectory")
     (cons :controller-state "rhand_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "right_finger_joint1" "right_finger_joint2") ))))
  #|
  (:start-grasp
  (arm &key (wait nil)) ;; TODO :arms is not implemented
  (unless (memq arm '(:larm :rarm))
  (error "you must specify arm ~A from ~A" (car args) '(:larm :rarm))
  (return-from :start-grasp nil))
  (send self :move-gripper arm -0.01 :effort 1.0 :wait wait)
  )
  (:stop-grasp
  (arm &key (wait nil)) ;; TODO :arms is not implemented
  (unless (memq arm '(:larm :rarm))
  (error "you must specify arm ~A from ~A" (car args) '(:larm :rarm))
  (return-from :stop-grasp nil))
  (send self :move-gripper arm 0.05 :effort 1.0 :wait wait)
  )
  (:get-table nil controller-table)
  (:move-gripper
  (arm pos &key (effort 25) (tm 500) (timeout 5000) (wait nil))  ;; TODO :arms is not implemented
  (let* ((start-time 0.1) ;; TODO fixed start-time
  (action (case arm
  (:rarm r-gripper-action)
  (:larm l-gripper-action)))
  (joint-names (case arm
  (:rarm (cdr (assoc :joint-names (car (send self :rhand-controller)))))
  (:larm (cdr (assoc :joint-names (car (send self :lhand-controller)))))))
  (goal (send action :make-goal-instance))
  (st (if (numberp start-time)
  (ros::time+ (ros::time-now) (ros::time start-time))
  start-time)))
  ;; send command
  (send goal :header :seq 1)
  (send goal :header :stamp st)
  (send goal :goal :trajectory :joint_names joint-names)
  (send goal :goal :trajectory :header :stamp st)
  (send goal :goal :trajectory :points
  (list (instance trajectory_msgs::JointTrajectoryPoint
  :init
  :positions (float-vector pos pos)
  :effort (float-vector effort effort)   ;; effort control
  :time_from_start (ros::time (/ tm 1000.0)))))
  (send self :spin-once)
  (send action :send-goal goal)
  (cond (wait (send action :wait-for-result :timeout timeout))))) ;; wait if needed
  (:gripper
  (&rest args)
  (when (eq (car args) :arms)
  (return-from :gripper
  (mapcar #'(lambda (x)
  (send self :gripper x (cadr args)))
  '(:larm :rarm))))
  (unless (memq (car args) '(:larm :rarm))
  (error "you must specify arm ~A from ~A" (car args) '(:larm :rarm))
  (return-from :gripper nil))
  (send self :state
  (intern
  (format nil "~A-~A" (string (car args)) (string (cadr args)))
  *keyword-package*)))
  (:gripper-state-callback
  (arm msg)
  (let* ((poss (send (send msg :actual) :positions))
  (vels (send (send msg :actual) :velocities)))
  (dolist (i (list
  (cons (format nil "~A-POSITION" (string arm))
  (* 1000.0 (/ (+ (elt poss 0) (elt poss 1)) 2)))
  (cons (format nil "~A-VELOCITY" (string arm))
  (* 1000.0 (/ (+ (elt vels 0) (elt vels 1)) 2)))))
  (send self :set-robot-state1 (intern (car i) *keyword-package*) (cdr i)))))
  |#
  ;;;;
  (:set-joint-pd-gain
   (joint-name pgain dgain)
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
   (dolist (j (send robot :joint-list))
     (send self :set-joint-pd-gain (send j :name) pgain dgain))
   )
  (:check-error ()
                (ros::spin-once "error_group")
                (or r-error l-error)
                )
  (:callback-rarm-error (msg)
                        (setq r-error (send msg :data))
                        )
  (:callback-larm-error (msg)
                        (setq l-error (send msg :data))
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
   (let (acts)
     (case
         arm
       (:rarm (setq acts (list r-gripper-stop-action)))
       (:larm (setq acts (list l-gripper-stop-action)))
       (:arms (setq acts (list r-gripper-stop-action l-gripper-stop-action)))
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
       (:rarm (setq acts (list r-gripper-homing-action)))
       (:larm (setq acts (list l-gripper-homing-action)))
       (:arms (setq acts (list r-gripper-homing-action l-gripper-homing-action)))
       )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-homing-action act
               pos (/ (* 1000 0.08) tm) :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  (:start-grasp
   (arm &key (width 0.0)(effort 80.0) (tm 500) (wait nil) (inner 0.005) (outer 0.06)) ;; TODO :arms is not implemented
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
   (arm &key (wait nil) (width 0.08)) ;; TODO :arms is not implemented
   (unless (memq arm '(:larm :rarm :arms))
     (error "you must specify arm ~A from ~A" (car args) '(:larm :rarm :arms))
     (return-from :stop-grasp nil))
   (send self :move-gripper arm width :tm 500 :wait wait)
   )
  (:move-gripper
   (arm pos &key (effort 25) (tm 500) (timeout 5000) (wait nil))  ;; TODO :arms is not implemented
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
               pos (/ (* 1000 0.08) tm) :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  )

;; grasp controller ...
(defun dual_panda-init ()
  (setq *ri* (instance dual_panda-robot-interface :init))
  (setq *robot* (dual_panda))
  )

#|
(send *ri* :set-all-joint-pd-gain 1000.0 5.0) ;; default
(send *ri* :set-all-joint-pd-gain  300.0 5.0) ;; hard
(send *ri* :set-all-joint-pd-gain   30.0 0.5) ;; soft
|#
