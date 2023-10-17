(require :robot-interface "package://pr2eus/robot-interface.l")

(ros::roseus-add-msgs "actionlib_msgs")
(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")

(defclass franka-common-interface
  :super robot-interface
  :slots (all-arms all-arm-aliases
          arm-error-p error-recovery-act
          gripper-grasp-actions gripper-move-actions gripper-homing-actions gripper-stop-actions
          )
  )
(defmethod franka-common-interface
  (:init
   (&rest args &key (robot nil)
          (joint-states-topic nil)
          ((:all-arms aa) nil)
          ((:all-arm-aliases aaa) nil)
          (error-topics nil)
          (error-topic-types nil)
          (error-recovery-action nil)
          (gripper-action-prefixes nil)
          &allow-other-keys)
   "Create robot interface for franka robot
Arguments specific to franka robot:
- all-arms : names of all arms (e.g., (list :rarm))
- all-arm-aliases : aliases of all arms (e.g., (list :arm))
- error-topics : topics used for getting error states of arms (e.g., (list \"/franka_state_controller/franka_states\"))
- error-topic-types : message types of error-topics (e.g., (list franka_msgs::FrankaState))
- error-recovery-action : action for error recovery (e.g., \"/franka_control/error_recovery\")
- gripper-action-prefixes : prefixes added to gripper action names (e.g., (list \"\"))
"
   (setq all-arms aa)
   (setq all-arm-aliases aaa)
   (unless (= (length all-arms) (length all-arm-aliases)
              (length error-topics) (length error-topic-types)
              (length gripper-action-prefixes))
     (error
       "all-arms, all-arm-aliases, error-topics, error-topic-types, and gripper-action-prefixes must have the same length")
     (return-from :init nil))
   (prog1
       (send-super* :init :robot robot
                    :joint-states-topic joint-states-topic
                    args)
     ;; for error recovery
     (ros::create-nodehandle "error_group")
     (setq arm-error-p (make-hash-table))
     (dotimes (i (length all-arms))
       (let ((arm (elt all-arms i)) (etype (elt error-topic-types i)))
         (sethash arm arm-error-p nil)
         (cond
           ((eq etype std_msgs::Bool)
            ;; franka_combined_control.launch (used in dual_panda) publishes has_error topic (800Hz)
            (ros::subscribe
              (elt error-topics i) std_msgs::Bool
              #'send self :callback-error arm
              1 :groupname "error_group"))
           ((eq etype franka_msgs::FrankaState)
            ;; franka_control.launch (used in single panda) does not publish has_error topic.
            ;; We use franka_states topic instead, but its frequency is lower (30Hz).
            ;; We do not use franka_combined_control.launch in single panda for backward compatibility.
            ;; position_joint_trajectory_controller, which we used in single panda so far, cannot be used on franka_combined_control.launch.
            ;; This is because
            ;; https://github.com/frankaemika/franka_ros/blob/0.10.1/franka_hw/src/franka_hw.cpp#L528-L529
            ;; does not exist in
            ;; https://github.com/frankaemika/franka_ros/blob/0.10.1/franka_hw/src/franka_combinable_hw.cpp#L26-L39.
            (ros::subscribe
              (elt error-topics i) franka_msgs::FrankaState
              #'send self :callback-franka-state arm
              1 :groupname "error_group"))
           (t
             (ros::ros-error "error-topic-types includes unsupported type")
             (ros::ros-error ":check-error will return meaningless value")))))
     (setq error-recovery-act (instance ros::simple-action-client :init
                                        error-recovery-action
                                        franka_msgs::ErrorRecoveryAction
                                        :groupname "error_group"
                                        ))
     ;; actions for gripper
     (setq gripper-grasp-actions (make-hash-table))
     (setq gripper-homing-actions (make-hash-table))
     (setq gripper-move-actions (make-hash-table))
     (setq gripper-stop-actions (make-hash-table))
     (dotimes (i (length all-arms))
       (let ((arm (elt all-arms i)) (prefix (elt gripper-action-prefixes i)))
         (sethash arm gripper-grasp-actions
                  (instance ros::simple-action-client :init
                            (format nil "~a/franka_gripper/grasp" prefix)
                            franka_gripper::GraspAction
                            :groupname groupname))
         (sethash arm gripper-homing-actions
                  (instance ros::simple-action-client :init
                            (format nil "~a/franka_gripper/homing" prefix)
                            franka_gripper::HomingAction
                            :groupname groupname))
         (sethash arm gripper-move-actions
                  (instance ros::simple-action-client :init
                            (format nil "~a/franka_gripper/move" prefix)
                            franka_gripper::MoveAction
                            :groupname groupname))
         (sethash arm gripper-stop-actions
                  (instance ros::simple-action-client :init
                            (format nil "~a/franka_gripper/stop"prefix)
                            franka_gripper::StopAction
                            :groupname groupname))))
     ))
  (:set-joint-pd-gain
   (joint-name pgain dgain &key (type :default-controller))
   "Set P gain and D gain of each joint.
This method works only when `type` includes a controller coming from effort_controllers/JointTrajectoryController and including `joint-name`.
"
   (if (send self :simulation-modep)
     (progn
       (ros::ros-warn ":set-joint-pd-gain is not implemented on simulation mode, always returns t")
       (return-from :set-joint-pd-gain t)))
   (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)) (set-p nil))
     (send req :config :doubles
           (list (instance dynamic_reconfigure::DoubleParameter :init
                           :name "p" :value pgain)
                 (instance dynamic_reconfigure::DoubleParameter :init
                           :name "d" :value dgain)))
     (dolist (fjt-name (mapcar #'(lambda (joint-param)
                                (cdr (assoc :controller-action joint-param)))
                            (send self type)))
       (let* ((cname (subseq fjt-name 0 (- (length fjt-name)
                                           (position #\/ (reverse fjt-name))
                                           1)))
              (srv-name (format nil "~a/gains/~a/set_parameters" cname joint-name)))
         ;; If fjt-name is "/dual_panda/dual_panda_effort_joint_trajectory_controller/follow_joint_trajectory",
         ;; cname will be "/dual_panda/dual_panda_effort_joint_trajectory_controller"
         (if (ros::wait-for-service srv-name 0)
           (progn
             (ros::service-call srv-name req)
             (setq set-p t)))))
     (if (not set-p)
       (progn
         (ros::ros-error "Setting PD gains of ~a via ~a failed" joint-name type)
         (ros::ros-error
           "~a may not include a controller coming from effort_controllers/JointTrajectoryController and including ~a"
           type joint-name)))
     ))
  (:set-all-joint-pd-gain
   (pgain dgain &key (type :default-controller))
   "Set P gain and D gain of all joints.
This method fully works only when each joint is included by a controller in `type` coming from effort_controllers/JointTrajectoryController.
"
   (dolist (j (send robot :joint-list))
     (send self :set-joint-pd-gain (send j :name) pgain dgain :type type))
   )
  (:check-error
   ()
   "Check if the robot has error.
If this method returns T, you must call :recover-error to move the robot.
"
   (ros::spin-once "error_group")
   (reduce #'(lambda (x y) (or x y))
           (mapcar #'(lambda (arm) (gethash arm arm-error-p)) all-arms))
   )
  (:callback-error
   (arm msg)
   (sethash arm arm-error-p (send msg :data))
   )
  (:callback-franka-state
   (arm msg)
   (sethash arm arm-error-p
            (not (= (send msg :robot_mode)
                    franka_msgs::FrankaState::*ROBOT_MODE_MOVE*)))
   )
  (:wait-recover-error
   ()
   (if (send self :simulation-modep)
     (progn
       (ros::ros-warn ":wait-recover-error is not implemented on simulation mode, always returns t")
       (return-from :wait-recover-error t)))
   (send error-recovery-act :wait-for-result)
   )
  (:recover-error
   (&key (wait t))
   "Recover from errors and reflexes.
Details: `ErrorRecoveryAction` part of https://frankaemika.github.io/docs/franka_ros.html#franka-control
"
   (if (send self :simulation-modep)
     (progn
       (ros::ros-warn ":recover-error is not implemented on simulation mode, always returns t")
       (return-from :recover-error t)))
   (let ((goal (instance franka_msgs::ErrorRecoveryActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     (send error-recovery-act :send-goal goal)
     (if wait (send self :wait-recover-error))
     ))
  (:expand-arm-alias
   (arm-alias)
   (let ((expanded
           (if arm-alias
             (let ((i (position arm-alias all-arm-aliases)))
               (if i
                 (elt all-arms i)
                 arm-alias))
             nil)))
     (if (memq expanded all-arms)
       expanded
       (progn
         (error "arm ~a does not exist" arm-alias)
         nil))
     ))
  ;; gripper action for real-controller
  (:send-gripper-grasp-action
   (arm width speed force &key (wait t) (inner 0.005) (outer 0.07))
   (let ((goal (instance franka_gripper::GraspActionGoal :init))
         (action (gethash (send self :expand-arm-alias arm) gripper-grasp-actions)))
     (send goal :header :stamp (ros::time-now))
     (send goal :goal :width width) ;; [m]
     (send goal :goal :speed speed) ;; [m/s]
     (send goal :goal :force force) ;; [N]
     (send goal :goal :epsilon :inner inner) ;; [m]
     (send goal :goal :epsilon :outer outer) ;; [m]
     ;;
     (if action
       (progn
         (send action :send-goal goal)
         (if wait (send action :wait-for-result))))
     ))
  (:send-gripper-homing-action
   (arm &key (wait t))
   (let ((goal (instance franka_gripper::HomingActionGoal :init))
         (action (gethash (send self :expand-arm-alias arm) gripper-homing-actions)))
     (send goal :header :stamp (ros::time-now))
     ;;
     (if action
       (progn
         (send action :send-goal goal)
         (if wait (send action :wait-for-result))))
     ))
  (:send-gripper-move-action
   (arm width speed &key (wait t))
   (let ((goal (instance franka_gripper::MoveActionGoal :init))
         (action (gethash (send self :expand-arm-alias arm) gripper-move-actions)))
     (send goal :header :stamp (ros::time-now))
     (send goal :goal :width width) ;; [m]
     (send goal :goal :speed speed) ;; [m/s]
     ;;
     (if action
       (progn
         (send action :send-goal goal)
         (if wait (send action :wait-for-result))))
     ))
  (:send-gripper-stop-action
   (arm &key (wait t))
   (let ((goal (instance franka_gripper::StopActionGoal :init))
         (action (gethash (send self :expand-arm-alias arm) gripper-stop-actions)))
     (send goal :header :stamp (ros::time-now))
     ;;
     (if action
       (progn
         (send action :send-goal goal)
         (if wait (send action :wait-for-result))))
     ))
  (:arm2arms
   (arm)
   (case arm
     (:arms all-arms)
     (t (list arm))
     ))
  (:send-gripper-action-method
   (arm actions method &rest args)
   (let ((action (gethash (send self :expand-arm-alias arm) actions)))
     (if action (send* action method args) nil))
   )
  (:gripper-action-postprocess
   (arm actions wait)
   (let ((arms (send self :arm2arms arm)) (wait-res t))
     (if wait
       (dolist (a arms)
         (setq wait-res
               (and (send self :send-gripper-action-method a actions :wait-for-result)
                    wait-res)))
       (send self :spin-once))
     (if (not wait-res)
       (progn
         (setq wait-res t)
         (dolist (a arms)
           (if (eq (send self :send-gripper-action-method a actions :get-state)
                   actionlib_msgs::GoalStatus::*pending*)
             (progn
               (ros::ros-error
                 "No goal exists for ~a, send goal to it first (:start-grasp, :stop-grasp, ...)"
                 (send self :send-gripper-action-method a actions :name))
               (setq wait-res nil))))
         (if (not wait-res)
           (return-from :gripper-action-postprocess nil))))
     (case arm
       (:arms
         (mapcar #'(lambda (a)
                     (send self :send-gripper-action-method a actions :get-result))
                 arms))
       (t
         (send self :send-gripper-action-method arm actions :get-result)))
     ))
  (:gripper-method-helper
   (action-sender actions arm wait)
   (let ((arms (send self :arm2arms arm)))
     (dolist (a arms)
       (send self action-sender a :wait nil))
     (send self :gripper-action-postprocess arm actions wait)
     ))
  (:stop-gripper
   (arm &key (wait nil))
   "Abort a running gripper action. This can be used to stop applying forces after grasping.
Details: `StopAction` part of https://frankaemika.github.io/docs/franka_ros.html#franka-gripper
"
   (if (send self :simulation-modep)
     (progn
       (ros::ros-warn ":stop-gripper is not implemented on simulation mode, always returns t")
       (return-from :stop-gripper t)))
   (send self :gripper-method-helper
         :send-gripper-stop-action gripper-stop-actions arm wait)
   )
  (:get-stop-gripper-result
   (arm &key (wait t))
   "Return result of :stop-gripper (`franka_gripper::StopActionResult`)"
   (send self :gripper-action-postprocess arm gripper-stop-actions wait)
   )
  (:homing-gripper
   (arm &key (wait nil))
   "Home the gripper and update the maximum width given the mounted fingers (i.e., calibrate & initialize the gripper).
Details: `HomingAction` part of https://frankaemika.github.io/docs/franka_ros.html#franka-gripper
"
   (if (send self :simulation-modep)
     (send self :stop-grasp arm :wait wait)
     (send self :gripper-method-helper
           :send-gripper-homing-action gripper-homing-actions arm wait)
     ))
  (:get-homing-gripper-result
   (arm &key (wait t))
   "Return result of :homing-gripper (`franka_gripper::HomingActionResult`)"
   (send self :gripper-action-postprocess arm gripper-homing-actions wait)
   )
  (:gripper
   (&rest args)
   "Get information of gripper
Arguments:
- arm : :arms or any element of all-arms and all-arm-aliases (e.g., :rarm)
- type : :position ([m])
Example: (send self :gripper :rarm :position) => 0.00
"
   (if (eq (car args) :arms)
     (return-from :gripper
                  (mapcar #'(lambda (x)
                              (send self :gripper x (cadr args)))
                          all-arms)))
   (let ((arm (send self :expand-arm-alias (car args))))
     (if arm
       (progn
         (send self :update-robot-state)
         (case (cadr args)
           (:position
             (* 0.001
                (apply #'+
                       (send-all
                         (remove nil (mapcar #'(lambda (jt)
                                                 (if (= (send jt :min-angle)
                                                        (send jt :max-angle))
                                                   nil jt))
                                             (send robot arm :gripper :joint-list)))
                         ;; Get joint list of gripper excluding fixed joints
                         :joint-angle))))))))
   )
  (:gripper-method-with-width-helper
   (action-sender actions arm wait width tm &rest args)
   (let ((arms (send self :arm2arms arm)))
     (dolist (a arms)
       (send* self action-sender a width
              (/ (abs (- (send self :gripper a :position) width))
                 (/ tm 1000.0))
              (append args (list :wait nil))))
     (send self :gripper-action-postprocess arm actions wait)
     ))
  (:start-grasp
   (arm &key (width 0.0) (effort 80.0) (tm 500) (wait nil) (inner 0.005) (outer 0.06))
   "Try to grasp at the desired `width` with the desired `effort` while closing with the desired speed calculated from `tm`.
Arguments:
- arm : :arms or any element of all-arms and all-arm-aliases (e.g., :rarm)
- width : target distance between the fingers [m]
- effort : target effort [N]
- tm : time to target [ms]. This will be converted to the movement speed
- wait : if this argument is T, this method waits until the movement finishes
- inner : lower admissible error of width. If this is violated, the gripper stops applying forces
- outer : upper admissible error of width. If this is violated, the gripper stops applying forces
          Details: https://github.com/ykawamura96/jsk_robot/pull/1#issuecomment-860324988
Details: `GraspAction` part of https://frankaemika.github.io/docs/franka_ros.html#franka-gripper
"
   (if (send self :simulation-modep)
     (send robot :start-grasp arm :width width)
     (send self :gripper-method-with-width-helper
           :send-gripper-grasp-action gripper-grasp-actions arm wait width tm
           effort :inner inner :outer outer)
     ))
  (:get-start-grasp-result
   (arm &key (wait t))
   "Return result of :start-grasp (`franka_gripper::GraspActionResult`)"
   (send self :gripper-action-postprocess arm gripper-grasp-actions wait)
   )
  (:stop-grasp
   (arm &key (wait nil) (width 0.08))
   "Open the gripper to the target `width` [m]"
   (send self :move-gripper arm width :tm 500 :wait wait)
   )
  (:get-stop-grasp-result
   (arm &key (wait t))
   "Return result of :stop-gripper (`franka_gripper::MoveActionResult`)"
   (send self :get-move-gripper-result arm :wait wait)
   )
  (:move-gripper
   (arm width &key (tm 500) (wait nil))
   "Move the gripper to the target `width` [m] while closing with the desired speed calculated from `tm` [ms].
Details: `MoveAction` part of https://frankaemika.github.io/docs/franka_ros.html#franka-gripper
"
   (if (send self :simulation-modep)
     (send robot :move-gripper arm width)
     (send self :gripper-method-with-width-helper
           :send-gripper-move-action gripper-move-actions arm wait width tm)
     ))
  (:get-move-gripper-result
   (arm &key (wait t))
   "Return result of :move-gripper (`franka_gripper::MoveActionResult`)"
   (send self :gripper-action-postprocess arm gripper-move-actions wait)
   )
  )

(provide :franka-common-interface)

#|
You can check current joint gains with rqt_reconfigure when you use effort_controllers/JointTrajectoryController.
Be careful changing them with :set-joint-pd-gain or :set-all-joint-pd-gain
|#
