(ros::roseus "spot")

(require "package://spoteus/spot-utils.l")
(require "package://pr2eus/robot-interface.l")

(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "spot_msgs")
(ros::roseus-add-srvs "spot_msgs")
(ros::roseus-add-srvs "control_msgs")

(defun call-trigger-service (srvname &key (wait nil))
  "Call std_srv/Trigger service"
  (let (r)
    (when (send *ri* :simulation-modep)
      (ros::ros-warn "call-trigger-service (~A)" srvname)
      (return-from call-trigger-service t))
    (if wait (ros::wait-for-service srvname))
    (setq r (ros::service-call srvname (instance std_srvs::TriggerRequest :init)))
    (ros::ros-debug "Call \"~A\" returns \"~A\"" srvname (send r :message))
    (send r :success)))

(defun call-set-bool-service (srvname data &key (wait nil))
  "Call std_srv/Trigger service"
  (let (r)
    (when (send *ri* :simulation-modep)
      (ros::ros-warn "call-set-bool-service (~A) ~A" srvname data)
      (return-from call-set-bool-service t))
    (if wait (ros::wait-for-service srvname))
    (setq r (ros::service-call srvname (instance std_srvs::SetBoolRequest :init :data data)))
    (ros::ros-info "Call \"~A\" returns \"~A\"" srvname (send r :message))
    (send r :success)))

(defun call-set-locomotion-service (srvname locomotion_hint &key (wait nil))
  "Call spot_msgs/SetLocomotion service"
  (let (r)
    (when (send *ri* :simulation-modep)
      (ros::ros-warn "call-set-locomotion-service (~A) ~A" srvname locomotion_hint)
      (return-from call-set-locomotion-service t))
    (if wait (ros::wait-for-service srvname))
    (setq r (ros::service-call srvname (instance spot_msgs::SetLocomotionRequest :init :locomotion_mode locomotion_hint)))
    (ros::ros-info "Call \"~A\" returns \"~A\"" srvname (send r :message))
    (send r :success)))

(defun call-dock-service (srvname dock_id &key (wait nil))
  "Call spot_msgs/Dock service"
  (let (r)
    (when (send *ri* :simulation-modep)
      (ros::ros-warn "call-dock-service (~A) ~A" srvname dock_id)
      (return-from call-dock-service t))
    (if wait (ros::wait-for-service srvname))
    (setq r (ros::service-call srvname (instance spot_msgs::DockRequest :init :dock_id dock_id)))
    (ros::ros-info "Call \"~A\" returns \"~A\"" srvname (send r :message))
    (send r :success)))

(defclass spot-interface
  :super robot-move-base-interface
  :slots (trajectory-cmd-action pick-object-in-image-client pick-object-in-image-feedback-msg)
  )

(defmethod spot-interface
  (:init
   (&rest args &key (trajectory-cmd-action-name "/spot/trajectory") &allow-other-keys)
   (prog1
       (send-super* :init :robot spot-robot :base-frame-id "base_link" :odom-topic "/odom_combined" :base-controller-action-name nil args)
     ;; check if spot_ros/driver.launch started
     (unless (or (send self :simulation-modep) (ros::wait-for-service "/spot/claim" 5))
       (ros::ros-error "could not communicate with robot, may be forget to roslaunch spot_driver driver.launch, or did not power on the robot"))
     ;; http://www.clearpathrobotics.com/assets/guides/melodic/spot-ros/ros_usage.html#view-the-robot
     ;; spot_msgs version 0.0.0
     (ros::subscribe "/spot/status/metrics" spot_msgs::Metrics #'send self :spot-status-metrics-callback :groupname groupname)
     (ros::subscribe "/spot/status/leases" spot_msgs::LeaseArray #'send self :spot-status-leases-callback :groupname groupname)
     (ros::subscribe "/spot/status/feet" spot_msgs::FootStateArray #'send self :spot-status-feet-callback :groupname groupname)
     (ros::subscribe "/spot/status/estop" spot_msgs::EStopStateArray #'send self :spot-status-estop-callback :groupname groupname)
     (ros::subscribe "/spot/status/wifi" spot_msgs::WiFiState #'send self :spot-status-wifi-callback :groupname groupname)
     (ros::subscribe "/spot/status/manipulator_state" spot_msgs::ManipulatorState #'send self :spot-status-manipulator-state-callback :groupname groupname)
     (ros::subscribe "/spot/status/power_state" spot_msgs::PowerState #'send self :spot-status-power-state-callback :groupname groupname)
     (ros::subscribe "/spot/status/battery_states" spot_msgs::BatteryStateArray #'send self :spot-status-battery-states-callback :groupname groupname)
     (ros::subscribe "/spot/status/behavior_faults" spot_msgs::BehaviorFaultState #'send self :spot-status-behavior-faults-callback :groupname groupname)
     (ros::subscribe "/spot/status/system_faults" spot_msgs::SystemFaultState #'send self :spot-status-system-faults-callback :groupname groupname)
     (ros::subscribe "/spot/status/feedback" spot_msgs::Feedback #'send self :spot-feedback-callback :groupname groupname)
     (setq trajectory-cmd-action (instance ros::simple-action-client :init
                                        trajectory-cmd-action-name spot_msgs::TrajectoryAction
                                        :groupname groupname))
     (setq pick-object-in-image-client (instance ros::simple-action-client :init
                                                 "/spot/pick_object_in_image" spot_msgs::PickObjectInImageAction))
     ))
  ;; (:default-controller  ()
  ;;  (append
  ;;   (send self :arm-controller)
  ;;   (send self :dummy-controller)
  ;;   ))
  (:default-controller () (send self :arm-controller))
  (:arm-controller ()
   (list
    (list
     (cons :controller-action "spot/arm_controller/follow_joint_trajectory")
     (cons :controller-state  "spot/arm_controller/follow_joint_trajectory/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (send-all (send robot :arm :joint-list) :name))
     )))
  (:set-impedance-param
   (&key (linear-stiffness #f(500 500 500))
	 (rotational-stiffness #f(60 60 60))
	 (linear-damping #f(2.5 2.5 2.5))
	 (rotational-damping #f(1.0 1.0 1.0)))
   "X is up, Z is forward"
   (let (m r)
     (labels
      ((v2v (v) (instance geometry_msgs::Vector3 :init :x (elt v 0) :y (elt v 1) :z (elt v 2))))
      (setq m (instance spot_msgs::SetArmImpedanceParamsRequest :init
			:linear_stiffness     (v2v linear-stiffness)
			:rotational_stiffness (v2v rotational-stiffness)
			:linear_damping       (v2v linear-damping)
			:rotational_damping   (v2v rotational-damping)))
      (setq r (ros::service-call "/spot/arm_impedance_parameters" m))
      (send r :success))))
  (:unstow-arm () (call-trigger-service "/spot/unstow_arm"))
  (:stow-arm () (call-trigger-service "/spot/stow_arm"))
  (:gripper-open () (call-trigger-service "/spot/gripper_open"))
  (:gripper-close () (call-trigger-service "/spot/gripper_close"))
  (:start-grasp
   ()
   (let (p s)
     (send self :gripper-close)
     (setq p (send *ri* :state :manipulator-state-gripper-open-percentage)
	   s (send *ri* :state :manipulator-state-is-gripper-holding-item))
     (ros::ros-info "gripper-open-percentage ~A" p)
     (ros::ros-info "manipulator-state-is-gripper-holding-item ~A" s)
     s))
  (:stop-grasp
   ()
   (let (p s)
     (send self :gripper-open)
     (setq p (send *ri* :state :manipulator-state-gripper-open-percentage)
	   s (send *ri* :state :manipulator-state-is-gripper-holding-item))
     (ros::ros-info "gripper-open-percentage ~A" p)
     (ros::ros-info "manipulator-state-is-gripper-holding-item ~A" s)
     s))
  (:pick-object-in-image
   (image-source x y &optional (max-duration 15))
   (let ((goal (instance spot_msgs::PickObjectInImageActionGoal :init)))
     (print (list image-source x y max-duration))
     (send goal :goal :image_source image-source)
     (send goal :goal :center (instance geometry_msgs::Point :init :x x :y y))
     (send goal :goal :max_duration (instance ros::duration :init max-duration))
     (send pick-object-in-image-client :send-goal goal :feedback-cb `(lambda (msg) (send ,self :pick-object-in-image-feedback-cb msg)))
     ))
  (:pick-object-in-image-feedback-cb
   (msg)
   (setq pick-object-in-image-feedback-msg msg))
  (:pick-object-in-image-feedback-msg
   (&rest args)
   (forward-message-to pick-object-in-image-feedback-msg args))
  (:pick-object-in-image-wait-for-result
   ()
   (send pick-object-in-image-client :wait-for-result)
   (send pick-object-in-image-client :get-result))
  (:pick-object-in-image-get-state
   ()
   (send pick-object-in-image-client :get-state))
  ;; (:dummy-controller ()
  ;;  (list
  ;;   (list
  ;;    (cons :controller-action "spot/dummy_controller/follow_joint_trajectory")
  ;;    (cons :controller-state  "spot/dummy_controller/follow_joint_trajectory/state")
  ;;    (cons :action-type control_msgs::FollowJointTrajectoryAction)
  ;;    (cons :joint-names (send-all (set-difference (send robot :joint-list) (send robot :arm :joint-list)) :name))
  ;;    )))
  (:spot-status-metrics-callback
   (msg)
   (send self :set-robot-state1 :metrics-distance (send msg :distance))
   (send self :set-robot-state1 :metrics-gaint-cycles (send msg :gait_cycles))
   (send self :set-robot-state1 :metrics-time-moving (send (send msg :time_moving) :to-sec))
   (send self :set-robot-state1 :metrics-electric-power (send (send msg :electric_power) :to-sec)))
  (:spot-status-leases-callback
   (msg)
   (dolist (resource (send msg :resources))
     (let ((r (string-upcase (send resource :resource))))
       (send self :set-robot-state1 (intern (format nil "LEASES-~A-RESOURCE" r) *keyword-package*)
             (send resource :lease :resource))
       (send self :set-robot-state1 (intern (format nil "LEASES-~A-EPOCH" r) *keyword-package*)
             (send resource :lease :epoch))
       (send self :set-robot-state1 (intern (format nil "LEASES-~A-SEQUENCE" r) *keyword-package*)
             (send resource :lease :sequence))
       (send self :set-robot-state1 (intern (format nil "LEASES-~A-CLIENT-NAME" r) *keyword-package*)
             (send resource :lease_owner :client_name))
       (send self :set-robot-state1 (intern (format nil "LEASES-~A-USER-NAME" r) *keyword-package*)
             (send resource :lease_owner :user_name)))))
  (:spot-status-feet-callback
   (msg)
   (send self :set-robot-state1 :feet
         (mapcar #'(lambda (state)
                     (list (cons :foot-position-rt-body (ros::tf-point->pos (send state :foot_position_rt_body)))
                           (cons :contact (case (send state :contact) (0 'unknown) (1 'made) (2 'lost))))) (send msg :states))))
  (:spot-status-estop-callback
   (msg)
   (dolist (state (send msg :estop_states))
     (let ((s (string-upcase (substitute #\- #\_ (send state :name)))))
       (send self :set-robot-state1 (intern (format nil "ESTOP-~A-NAME" s) *keyword-package*)
             (send state :name))
       (send self :set-robot-state1 (intern (format nil "ESTOP-~A-TYPE" s) *keyword-package*)
             (case (send state :type) (0 'unknown) (1 'hardware) (2 'software)))
       (send self :set-robot-state1 (intern (format nil "ESTOP-~A-STATE" s) *keyword-package*)
             (case (send state :state) (0 'unknown) (1 'estopped) (2 'not-estopped)))
       (send self :set-robot-state1 (intern (format nil "ESTOP-~A-STATE-DESCRIPTION" s) *keyword-package*)
             (send state :state_description)))))
  (:spot-status-wifi-callback
   (msg)
   (send self :set-robot-state1 :wifi-current-mode
         (case (send msg :current_mode)
               (0 'unknown) (1 'access-point) (2 'client)))
   (send self :set-robot-state1 :wifi-essid (send msg :essid)))
  (:spot-status-manipulator-state-callback
   (msg)
   (send self :set-robot-state1 :manipulator-state-gripper-open-percentage (send msg :gripper_open_percentage))
   (send self :set-robot-state1 :manipulator-state-is-gripper-holding-item (send msg :is_gripper_holding_item))
   (send self :set-robot-state1 :manipulator-state-estimated-end-effector-force-in-hand
	 (let ((m (send msg :estimated_end_effector_force_in_hand)))
	   (float-vector (send m :x) (send m :y) (send m :z))))
   (send self :set-robot-state1 :manipulator-state-stow-state
         (case (send msg :stow_state) (0 'unknown) (1 'stowed) (2 'deployed)))
   (send self :set-robot-state1 :manipulator-state-velocity-of-hand-in-vision
	 (let ((m (send msg :velocity_of_hand_in_vision)))
	   (float-vector (send m :linear :x) (send m :linear :y) (send m :linear :z)
			 (send m :angular :x) (send m :angular :y) (send m :angular :z))))
   (send self :set-robot-state1 :manipulator-state-velocity-of-hand-in-odom
	 (let ((m (send msg :velocity_of_hand_in_odom)))
	   (float-vector (send m :linear :x) (send m :linear :y) (send m :linear :z)
			 (send m :angular :x) (send m :angular :y) (send m :angular :z))))
   (send self :set-robot-state1 :manipulator-carry-state
	 (case (send msg :carry_state) (0 'unknown) (1 'not-carriable) (2 'carriable) (3 'carriable-and-stowable)))
   )
  (:spot-status-power-state-callback
   (msg)
   (send self :set-robot-state1 :power-state-motor-power-state
         (case (send msg :motor_power_state)
               (0 'unknown) (1 'off) (2 'on) (3 'powering-on) (4 'powering-off) (5 'error)))
   (send self :set-robot-state1 :power-state-shore-power-state
         (case (send msg :shore_power_state)
               (0 'unknown-shore-power) (1 'on-shore-power) (2 'off-shore-power)))
   (send self :set-robot-state1 :power-state-locomotion-charge-percentage (send msg :locomotion_charge_percentage))
   (send self :set-robot-state1 :power-state-locomotion-estimated-runtime (send (send msg :locomotion_estimated_runtime) :to-sec)))
  (:spot-status-battery-states-callback
   (msg)
   (dolist (state (send msg :battery_states))
     (let ((i 1))
       (send self :set-robot-state1 (intern (format nil "BATTERY-STATES-~A-IDENTIFIER" i) *keyword-package*)
             (send state :identifier))
       (send self :set-robot-state1 (intern (format nil "BATTERY-STATES-~A-CHARGE-PERCENTAGE" i) *keyword-package*)
             (send state :charge_percentage))
       (send self :set-robot-state1 (intern (format nil "BATTERY-STATES-~A-ESTIMATED-RUNTIME" i) *keyword-package*)
             (send (send state :estimated_runtime) :to-sec))
       (send self :set-robot-state1 (intern (format nil "BATTERY-STATES-~A-CURRENT" i) *keyword-package*)
             (send state :current))
       (send self :set-robot-state1 (intern (format nil "BATTERY-STATES-~A-VOLTAGE" i) *keyword-package*)
             (send state :voltage))
       (send self :set-robot-state1 (intern (format nil "BATTERY-STATES-~A-TEMPERATURES" i) *keyword-package*)
             (send state :temperatures))
       (send self :set-robot-state1 (intern (format nil "BATTERY-STATES-~A-STATUS" i) *keyword-package*)
             (case (send state :status) (0 'unknown) (1 'missing) (2 'charging) (3 'discharging) (4 'booting)))
       (setq i (+ i 1))
       )))
  (:spot-status-behavior-faults-callback
   (msg)
   (send self :set-robot-state1 :behavior-faults
         (mapcar #'(lambda (fault)
                     (list (cons :behavior-fault-id (send fault :behavior_fault_id))
                           (cons :cause (case (send fault :cause) (0 'unknown) (1 'fall) (2 'hardware)))
                           (cons :status (case (send fault :status) (0 'unknown) (1 'clearable) (2 'unclearable)))))
                     (send msg :faults))))
  (:spot-status-system-faults-callback
   (msg)
   (dolist (fault (send msg :faults))
     (let ((s (string-upcase (substitute #\- #\_ (send fault :name)))))
       (send self :set-robot-state1 (intern (format nil "SYSTEM-FAULTS-~A-NAME" s) *keyword-package*)
             (send fault :NAME))
       (send self :set-robot-state1 (intern (format nil "SYSTEM-FAULTS-~A-DURATION" s) *keyword-package*)
             (send (send fault :duration) :to-sec))
       (send self :set-robot-state1 (intern (format nil "SYSTEM-FAULTS-~A-CODE" s) *keyword-package*)
             (send fault :code))
       (send self :set-robot-state1 (intern (format nil "SYSTEM-FAULTS-~A-UID" s) *keyword-package*)
             (send fault :uid))
       (send self :set-robot-state1 (intern (format nil "SYSTEM-FAULTS-~A-ERROR_MESSAGE" s) *keyword-package*)
             (send fault :error_message))
       (send self :set-robot-state1 (intern (format nil "SYSTEM-FAULTS-~A-ATTRIBUTES" s) *keyword-package*)
             (send fault :attributes))
       (send self :set-robot-state1 (intern (format nil "SYSTEM-FAULTS-~A-SEVERITY" s) *keyword-package*)
             (case (send fault :severity) (0 'unknown) (1 'info) (2 'warn) (3 'critical))))))
  (:spot-feedback-callback
   (msg)
   (send self :set-robot-state1 :feedback-standing (send msg :standing))
   (send self :set-robot-state1 :feedback-sitting (send msg :sitting))
   (send self :set-robot-state1 :feedback-moving (send msg :moving))
   (send self :set-robot-state1 :feedback-serial-number (send msg :serial_number))
   (send self :set-robot-state1 :feedback-species (send msg :species))
   (send self :set-robot-state1 :feedback-version (send msg :version))
   (send self :set-robot-state1 :feedback-nickname (send msg :nickname))
   (send self :set-robot-state1 :feedback-computer-serial-number(send msg :computer_serial_number)))
  (:state
   (&rest args)
   "use :metrics, :leases :feet, :estop, :wifi, :power-state, :battery-states, :behvior-faults, :system-fault, :feedback to get spot status, you can also acess to the specific data by concatenating these method name + key value, for example :metrics-time-moving"
   (prog1
       (send-message* self robot-interface :state args)
     (flet ((gen-status
             (key)
             (mapcan #'(lambda (x) (if (substringp (string key) (string (car x))) (list (cons (intern (subseq (string (car x)) (1+ (length (string key)))) *keyword-package*) (cdr x))))) robot-state)))
     (case (car args)
       (:metrics (return-from :state (gen-status :metrics)))
       (:leases  (return-from :state (gen-status :leases)))
       ;; (:feet    (return-from :state (gen-status :feet)))
       (:estop  (return-from :state (gen-status :estop)))
       (:wifi   (return-from :state (gen-status :wifi)))
       (:manipulator-state     (return-from :state (gen-status :manipulator-state)))
       (:power-state     (return-from :state (gen-status :power-state)))
       (:battery-states  (return-from :state (gen-status :battery-states)))
       ;; (:behavior-faults (return-from :state (gen-status :behavior-faults)))
       (:system-faults   (return-from :state (gen-status :system-faults)))
       (:feedback        (return-from :state (gen-status :feedback)))
       (:angle-vector
	(return-from :state (send robot :angle-vector)))
       (:worldcoords
	(unless joint-action-enable
	  (return-from :state (send self :worldcoords)))
        (return-from :state (send *tfl* :lookup-transform "vision" base-frame-id (ros::time 0))))))))
  ;;
  (:estop-gentle () (call-trigger-service "/spot/estop/gentle"))
  (:estop-hard () (call-trigger-service "/spot/estop/hard"))
  (:claim ()
   "Claim the robot control"
   (let ((client (send self :state :leases-body-client-name)))
     (if (or (null client) (string= client ""))
	 (call-trigger-service "/spot/claim")
       (ros::ros-warn "robot is already claimed by ~A" client))))
  (:release () "Relase the robot control" (call-trigger-service "/spot/release"))
  (:power-on ()
   "Power on the robot"
   (if (send *ri* :simulation-modep)
       (send self :set-robot-state1 :power-state-motor-power-state 'on)
     (call-trigger-service "/spot/power_on")))
  (:power-off
   ()
   "Power off the robot"
   (if (send *ri* :simulation-modep)
       (send self :set-robot-state1 :power-state-motor-power-state 'off)
     (call-trigger-service "/spot/power_off")))
  (:self-right () (call-trigger-service "/spot/self_right"))
  (:stand () "Stand the robot up" (call-trigger-service "/spot/stand"))
  (:sit () "Sit the robot down" (call-trigger-service "/spot/sit"))
  (:stop () "Stop the robot in place with minimal motion" (call-trigger-service "/spot/stop"))
  (:set-locomotion-mode (locomotion-hint) "Set locomotion mode" (call-set-locomotion-service "/spot/locomotion_mode" locomotion-hint))
  (:set-stair-mode (is-stair-mode) "Set stair mode" (call-set-bool-service "/spot/stair_mode" is-stair-mode))
  ;;
  (:dock (dock-id) "Dock the robot" (call-dock-service "/spot/dock" dock-id))
  (:undock
   ()
   "Undock the robot"
   (send self :power-on)
   (call-trigger-service "/spot/undock"))
  ;;
  (:send-cmd-vel-raw
   (x y d &key (topic-name "/spot/cmd_vel"))
   (when (send self :simulation-modep)
     (return-from :send-cmd-vel-raw t))
    (unless (ros::get-topic-publisher topic-name)
      (ros::advertise topic-name geometry_msgs::Twist 1)
      (unix:sleep 1))
    (let ((msg (instance geometry_msgs::Twist :init)))
      (send msg :linear :x x)
      (send msg :linear :y y)
      (send msg :angular :z d)
      (ros::publish topic-name msg)))
  (:go-velocity
    (x y d ;; [m/sec] [m/sec] [rad/sec]
       &optional (msec 1000) ;; msec is total animation time [msec]
       &key (stop t) (wait t))
    "control the robot velocity x([m/sec]) y([m/sec]) d([rad/sec]) msec([msec]). msec is the time to travel."
    (unless wait
      (ros::ros-error ":go-velocity without wait is unsupported")
      (return-from :go-velocity nil))
    (ros::rate 100)
    (let ((start-time (ros::time-now)))
      (while (and (ros::ok)
                  (< (* 1000.0 (send (ros::time- (ros::time-now) start-time) :to-sec)) msec))
        (send self :spin-once)
        (send self :send-cmd-vel-raw x y d)
        (ros::sleep)))
    (when stop
      (send self :send-cmd-vel-raw 0 0 0))
    (ros::rate 10)
    t)
  (:go-pos
    (x y &optional (d 0) &key (timeout 10) (wait t)) ;; [m] [m] [degree]
    "move robot toward (x, y, d) (units are m, m and degrees respectively)."
    ;;
    (setq trajectory-cmd-goal-msg (instance spot_msgs::TrajectoryGoal :init))
    (send trajectory-cmd-goal-msg :target_pose :header :stamp (ros::time-now))
    (send trajectory-cmd-goal-msg :target_pose :header :frame_id "body")
    (send trajectory-cmd-goal-msg :target_pose :pose :position :x x)
    (send trajectory-cmd-goal-msg :target_pose :pose :position :y y)
    (send trajectory-cmd-goal-msg :target_pose :pose :orientation :z (sin (/ (deg2rad d) 2)))
    (send trajectory-cmd-goal-msg :target_pose :pose :orientation :w (cos (/ (deg2rad d) 2)))
    (send trajectory-cmd-goal-msg :duration :data (ros::time timeout))
    ;;
    (send trajectory-cmd-action :send-goal trajectory-cmd-goal-msg)
    (if wait
        (send trajectory-cmd-action :wait-for-result)
        )
    )
  (:go-pos-wait
    (x y &optional (d 0) (timeout 10)) ;; [m] [m] [degree]
    "move robot toward (x, y, d) (units are m, m and degrees respectively). and wait"
    (send self :go-pos x y d :timeout timeout :wait t)
    )
  (:go-pos-no-wait
    (x y &optional (d 0) (timeout 10)) ;; [m] [m] [degree]
    "move robot toward (x, y, d) (units are m, m and degrees respectively)."
    (send self :go-pos x y d :timeout timeout :wait nil)
    )
  (:body-pose
   ;; pose expected to get 3 elements float-vector (r p y), or eus coords.
    (pose &key (topic-name "/spot/body_pose"))
    (when (send self :simulation-modep)
      (return-from :body-pose t))
    (unless (ros::get-topic-publisher topic-name)
      (ros::advertise topic-name geometry_msgs::Pose 1)
      (unix:sleep 1))
    (if (or (vectorp pose) (listp pose))
        (progn
          (ros::ros-debug "Got r p y float-vector or list as args.")
          (let ((pose-msg (instance geometry_msgs::Pose :init))
                (quaternion-msg (send self :create-quaternion-msg-from-rpy (elt pose 0) (elt pose 1) (elt pose 2))))
            (send pose-msg :orientation quaternion-msg)
            (ros::publish topic-name pose-msg)))
        (progn
          (ros::ros-debug "Got coords variable as args.")
          (let ((pose-msg (ros::coords->tf-pose pose)))
            (ros::publish topic-name pose-msg))))
    (send self :stand))
  (:find-waypoint-position-from-id
   (id ids)
   (let (ret)
     (setq ret (position id ids :test #'string=))
     ret))
  (:find-waypoint-id-from-position
   (index ids)
   "return waypint id from position, if you set -1 to index, it returns last waypoint"
   (let (ret)
     (if (< index 0)
	 (setq index (+ (length ids) index)))
     (if (< index (length ids))
	 (setq ret (elt ids index)))
     ret))
  (:set-localization-fiducial
   ()
   "initial the localization for autowalk based on the fiducial marker"
   (let (r)
     (setq r (ros::service-call "/spot/set_localization_fiducial"
                                (instance spot_msgs::SetLocalizationFiducialRequest :init)))
     (ros::ros-info "Call \"/spot/set_localization_fiducial\" returns \"~A\"" (send r :message))
     t))
  (:set-localization-waypoint
   (init-waypoint)
   "initial the localization for waypoint id in the graph"
   (let (r)
     (if (numberp init-waypoint)
         (prog1
             (setq ids (send self :list-graph))
           (setq init-waypoint (send self :find-waypoint-id-from-position init-waypoint ids))))
     (setq r (ros::service-call "/spot/set_localization_waypoint"
                                (instance spot_msgs::SetLocalizationWaypointRequest :init :waypoint_id init-waypoint)))
     (ros::ros-info "Call \"/spot/set_localization_waypoint\" returns \"~A\"" (send r :message))
     t))
  (:start-recording
    ()
    "Start AutoWalk recording"
    (call-trigger-service "/spot/start-recording"))
  (:stop-recording
    ()
    "Stop AutoWalk recording"
    (call-trigger-service "/spot/stop-recording"))
  (:upload-graph
    (path &key (initial-localization-fiducial t) (wait nil))
    "upload graph for autowalk"
    (let (r
          (srvname "/spot/upload_graph"))
      (if wait (ros::wait-for-service srvname))
      (setq r (ros::service-call srvname
                                 (instance spot_msgs::UploadGraphRequest :init :upload_filepath path)))
      (ros::ros-info "Call \"~A\" returns .. \"~A\"" srvname (send r :message))
      (if initial-localization-fiducial (send self :initial-localization-fiducial))
      (send r :success)))
  (:download-graph
    (path &key (wait nil))
    "Download AutoWalk graph from robot"
    (let (r)
      (if wait (ros::wait-for-service srvname))
      (setq r (ros::service-call srvname
                                 (instance spot_msgs::DownloadGraphRequest :init :download_filepath path)))
      (ros::ros-info "Call \"~A\" returns... \"~A\"" srvname (send r :message))
      (send r :success)
      ))
  (:list-graph
   ()
   "list up the waypoint (a string type hash id) list in the uploaded graph"
   (let (r)
     (setq r (ros::service-call "/spot/list_graph"
                                (instance spot_msgs::ListGraphRequest :init)))
     (ros::ros-info "Call \"/spot/list_graph\" returns ..")
     (dolist (id (send r :waypoint_ids))
       (ros::ros-info "  \"~A\"" id))
     (send r :waypoint_ids)))
  (:clear-graph
    ()
    "Clear the AutoWalk graph in the robot."
    (let ()
      (call-trigger-service "/spot/clear_graph")
      ))
  (:navigate-to
   (navigate-to &key (initial-localization-waypoint nil))
   (let (ids c goal ret)
     (setq ids (send self :list-graph))
     (if (numberp navigate-to)
	 (setq navigate-to (send self :find-waypoint-id-from-position navigate-to ids)))
     (if (numberp initial-localization-waypoint)
	 (setq initial-localization-waypoint (send self :find-waypoint-id-from-position initial-localization-waypoint ids)))
     (setq c (instance ros::simple-action-client :init
		       "/spot/navigate_to" spot_msgs::NavigateToAction))
     (send c :wait-for-server)
     (ros::ros-info "initial waypoit ~3D/~3D ~A"
		    (if initial-localization-waypoint
			(position initial-localization-waypoint ids :test #'string=)
		      -1)
		    (length ids)
		    initial-localization-waypoint)
     (ros::ros-info "   goal waypoit ~3D/~3D ~A"
		    (position navigate-to ids :test #'string=) (length ids) navigate-to)
     (setq goal (instance spot_msgs::NavigateToActionGoal :init))
     (send goal :goal :id_navigate_to navigate-to)
     (send c :send-goal goal :feedback-cb #'(lambda (msg) (let ((id (send msg :feedback :waypoint_id))) (ros::ros-info "~A/~A ~A" (position id ids :test #'string=) (length ids) id))))
     (setq ret (send c :wait-for-result))
     (ros::ros-info "~A ~A" ret (send (send c :get-result) :message))
     ret))
  ;; These are the methods to calculate rpy -> quaternion.
  ;; You can use ros::create-quaternion-from-rpy after https://github.com/jsk-ros-pkg/jsk_roseus/pull/662 was merged and released.
  (:create-quaternion-from-rpy
   (roll pitch yaw)                     ;; return [w x y z]
   (let ((sin-roll  (sin (* roll  0.5))) (cos-roll  (cos (* roll  0.5)))
         (sin-pitch (sin (* pitch 0.5))) (cos-pitch (cos (* pitch 0.5)))
         (sin-yaw   (sin (* yaw   0.5))) (cos-yaw   (cos (* yaw   0.5))))
     (float-vector (+ (* cos-roll cos-pitch cos-yaw) (* sin-roll sin-pitch sin-yaw))
                   (- (* sin-roll cos-pitch cos-yaw) (* cos-roll sin-pitch sin-yaw))
                   (+ (* cos-roll sin-pitch cos-yaw) (* sin-roll cos-pitch sin-yaw))
                   (- (* cos-roll cos-pitch sin-yaw) (* sin-roll sin-pitch cos-yaw)))))
  (:create-quaternion-msg-from-rpy
   (roll pitch yaw)
   (let* ((q (send self :create-quaternion-from-rpy roll pitch yaw))
          (qx (elt q 1)) (qy (elt q 2)) (qz (elt q 3)) (qw (elt q 0)))
     (instance geometry_msgs::quaternion :init :x qx :y qy :z qz :w qw)))
  )

(defun spot-init (&optional (create-viewer))
  (unless (boundp '*spot*) (spot) (send *spot* :reset-pose))
  (unless (ros::ok) (ros::roseus "spot_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)
  (send *ri* :claim)
  (while (member (send *ri* :state :power-state-motor-power-state) (list 'off nil))
    (unix::sleep 1)
    (ros::ros-info "powering on...")
    (send *ri* :power-on))
  ;;
  (unless (every #'(lambda (x) (eq x 'made)) (mapcar #'(lambda (x) (cdr (assoc :contact x))) (send *ri* :state :feet)))
      (ros::ros-info "run (send *ri* :stand) to stand the robot"))

  (when create-viewer (objects (list *spot*)))
  )