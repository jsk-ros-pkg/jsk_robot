;; Detachable hand client shared by ROS 1 (dynamixel_general_hw) and ROS 2 (ros2_control)

(defvar *detachable-hand-ros-version*
  (let ((v (unix:getenv "ROS_VERSION")))
    (if (and v (string= v "2")) 2 1)))

(defun detachable-hand-ros2-p () (= *detachable-hand-ros-version* 2))

(defun detachable-hand-ros-value (str)
  (symbol-value (read-from-string str)))

(if (detachable-hand-ros2-p)
    (require "package://roseus/euslisp/actionlib2.l")
  (require "package://roseus/euslisp/actionlib.l"))

(unless (ros::ok)
  (ros::roseus "detachable_hand_interface"))

(if (detachable-hand-ros2-p)
    (progn
      (ros::load-ros-package "action_msgs")
      (ros::load-ros-package "builtin_interfaces")
      (ros::load-ros-package "diagnostic_msgs")
      (ros::load-ros-package "std_srvs"))
  (ros::load-ros-package "actionlib_msgs"))
(ros::load-ros-package "control_msgs")
(ros::load-ros-package "sensor_msgs")
(ros::load-ros-package "std_msgs")
(ros::load-ros-package "trajectory_msgs")

;; GoalStatus constants: actionlib_msgs (ROS 1) / action_msgs (ROS 2)
(defvar *detachable-hand-status-succeeded*
  (detachable-hand-ros-value
   (if (detachable-hand-ros2-p) "action_msgs::GoalStatus::*STATUS_SUCCEEDED*"
     "actionlib_msgs::GoalStatus::*succeeded*")))
(defvar *detachable-hand-status-executing*
  (detachable-hand-ros-value
   (if (detachable-hand-ros2-p) "action_msgs::GoalStatus::*STATUS_EXECUTING*"
     "actionlib_msgs::GoalStatus::*active*")))
(defvar *detachable-hand-status-canceled*
  (detachable-hand-ros-value
   (if (detachable-hand-ros2-p) "action_msgs::GoalStatus::*STATUS_CANCELED*"
     "actionlib_msgs::GoalStatus::*preempted*")))
(defvar *detachable-hand-active-statuses*
  (mapcar #'detachable-hand-ros-value
          (if (detachable-hand-ros2-p)
              (list "action_msgs::GoalStatus::*STATUS_ACCEPTED*"
                    "action_msgs::GoalStatus::*STATUS_EXECUTING*"
                    "action_msgs::GoalStatus::*STATUS_CANCELING*")
            (list "actionlib_msgs::GoalStatus::*pending*"
                  "actionlib_msgs::GoalStatus::*active*"))))

(defun detachable-hand-duration (tm)
  "trajectory_msgs time_from_start of tm [s]"
  (if (detachable-hand-ros2-p)
      (let* ((tm-float (float tm))
             (tm-sec (floor tm-float))
             (tm-nsec (floor (* (- tm-float tm-sec) 1000000000))))
        (instance (detachable-hand-ros-value "builtin_interfaces::Duration")
                  :init :sec tm-sec :nanosec tm-nsec))
    (ros::time tm)))

(defun detachable-hand-xml-attr (line key)
  "value of key=\"...\" in a line of XML"
  (let ((k (format nil " ~A=\"" key)))
    (dotimes (i (max 0 (- (length line) (length k))))
      (when (string= k (subseq line i (+ i (length k))))
        (let ((start (+ i (length k))))
          (return-from detachable-hand-xml-attr
            (subseq line start (position #\" line :start start))))))
    nil))

(defvar *detachable-hand-named-positions* nil)
(defun detachable-hand-named-position (side tool state)
  "motor angle of <group_state name=state group=side_tool> in srdf/<side>.srdf"
  (let ((cache (assoc side *detachable-hand-named-positions* :test #'string=)))
    (unless cache
      (let ((path (ros::resolve-ros-path
                   (format nil "package://dynamixel_detachable_hand/srdf/~A.srdf" side)))
            positions group name line)
        (with-open-file (f path)
          (while (setq line (read-line f nil nil))
            (cond ((substringp "<group_state" line)
                   (setq name (detachable-hand-xml-attr line "name")
                         group (detachable-hand-xml-attr line "group")))
                  ((and name (substringp " value=" line))
                   (push (cons (format nil "~A/~A" group name)
                               (float (read-from-string (detachable-hand-xml-attr line "value"))))
                         positions)
                   (setq name nil)))))
        (setq cache (cons side positions))
        (push cache *detachable-hand-named-positions*)))
    (or (cdr (assoc (format nil "~A_~A/~A" side tool state) (cdr cache) :test #'string=))
        (error "no <group_state name=~A group=~A_~A> in srdf/~A.srdf" state side tool side))))

(defun detachable-hand-split-lines (text)
  (let ((lines nil) (start 0) pos)
    (while (setq pos (position #\newline text :start start))
      (push (subseq text start pos) lines)
      (setq start (1+ pos)))
    (push (subseq text start) lines)
    (nreverse lines)))

(defun detachable-hand-set-stamp-now (header)
  (let ((now (ros::time-now)))
    (if (detachable-hand-ros2-p)
        (progn
          (send header :stamp :sec (send now :sec))
          (send header :stamp :nanosec (send now :nsec)))
      (send header :stamp now))
    header))

(defclass detachable-hand-interface
  :super propertied-object
  :slots (side action-client- move-state- joint-states- groupname controller-action
          tool-name hand-joint gripper-clients
          execution-mode fallback-warned-p simulation-started-p
          current-hand-position current-detach-position hand-effort
          simulation-target-sequence simulation-timer-sequence
          simulation-current-time simulation-previous-position))

(defmethod detachable-hand-interface
  (:init (&key ((:side sd) "lhand")
               ((:groupname grp) nil)
               ((:execution-mode em) :auto)
               ;; ROS 2 discovery can take longer than 1 s
               ((:server-timeout server-timeout) (if (detachable-hand-ros2-p) 5.0 1.0)))
    (setq side sd)
    (setq groupname (or grp side))
    (setq controller-action
          (format nil "/~A/position_joint_trajectory_controller/follow_joint_trajectory" groupname))
    (setq tool-name nil)
    (setq hand-joint (format nil "~A_joint" side))
    (setq gripper-clients (make-hash-table :test #'equal))
    (setq execution-mode em)
    (setq fallback-warned-p nil)
    (setq simulation-started-p nil)
    (setq current-hand-position 0.0)
    (setq current-detach-position 0.0)
    ;; ROS 1: Goal Current limit of the hand motor set by hand_control.launch
    (setq hand-effort
          (float (or (and (not (detachable-hand-ros2-p))
                          (ros::get-param (format nil "/~A/hand_effort_limit" groupname)))
                     (send self :effort-limit))))
    (setq simulation-target-sequence nil)
    (setq simulation-timer-sequence nil)
    (setq simulation-current-time 0.0)
    (setq simulation-previous-position nil)
    (setq move-state- *detachable-hand-status-succeeded*)
    (setq joint-states- (make-hash-table :test #'equal))
    (when groupname
      (ros::create-nodehandle groupname))
    (if (detachable-hand-ros2-p)
        (send self :init-tool-state)
      (setq action-client-
            (instance ros::simple-action-client :init
                      controller-action control_msgs::FollowJointTrajectoryAction
                      :groupname groupname)))
    (ros::advertise (send self :effort-command-topic) std_msgs::Float64MultiArray 1)
    (ros::subscribe (send self :joint-states-publish-topic) sensor_msgs::JointState
                    #'send self :joint-states-callback 1 :groupname groupname)
    (when (eq execution-mode :kinematic-simulator)
      (send self :start-kinematic-simulator))
    (when (eq execution-mode :auto)
      (send self :ensure-controller-or-simulator server-timeout))
    self)

  ;; side specific defaults, overridden by lhand-interface / rhand-interface
  (:detach-torque-constant () 1.15)
  (:detach-current-ma () 400.0)
  (:effort-limit () 0.5)
  (:gripper-effort () 0.5)
  (:gripper-close-effort () 0.5)
  (:gripper-open-position () (send self :named-position "gripper" "open"))
  (:gripper-close-position () (send self :named-position "gripper" "close"))
  (:gripper-open-time () 0.25)

  (:side () side)
  (:named-position (tool state) (detachable-hand-named-position side tool state))
  (:tool () tool-name)
  (:hand-joint-name () hand-joint)

  ;; ROS 2: the attached tool decides the joint (<side>_<tool>_joint) and the controller
  (:init-tool-state ()
    (ros::subscribe (format nil "/~A/tool_state" groupname)
                    (detachable-hand-ros-value "diagnostic_msgs::DiagnosticArray")
                    #'send self :tool-state-callback 1 :groupname groupname)
    (let ((srv (format nil "/~A/get_tool_state" groupname)))
      (when (ros::wait-for-service srv 5)
        (let ((res (ros::service-call srv (instance (detachable-hand-ros-value "std_srvs::TriggerRequest") :init))))
          (when res (send self :parse-tool-state (send res :message)))))))
  (:parse-tool-state (text)
    ;; "key: value" lines of get_tool_state
    (let (attached tool joint)
      (dolist (line (detachable-hand-split-lines text))
        (let ((pos (position #\: line)))
          (when pos
            (let ((key (string-trim " " (subseq line 0 pos)))
                  (value (string-trim " " (subseq line (1+ pos)))))
              (cond ((string= key "attached") (setq attached (string= value "true")))
                    ((string= key "tool") (setq tool value))
                    ((string= key "command_joint") (setq joint value)))))))
      (send self :set-tool (and attached tool) joint)))
  (:tool-state-callback (msg)
    (dolist (status (send msg :status))
      (let (attached tool joint)
        (dolist (kv (send status :values))
          (cond ((string= (send kv :key) "attached") (setq attached (string= (send kv :value) "true")))
                ((string= (send kv :key) "tool") (setq tool (send kv :value)))
                ((string= (send kv :key) "command_joint") (setq joint (send kv :value)))))
        (when tool (send self :set-tool (and attached tool) joint)))))
  (:set-tool (tool joint)
    (setq tool-name tool)
    (setq hand-joint (if (and joint (> (length joint) 0)) joint (format nil "~A_joint" side)))
    (setq action-client- (and tool-name (send self :gripper-client tool-name))))
  (:gripper-client (tool)
    (or (gethash tool gripper-clients)
        (sethash tool gripper-clients
                 (instance ros::simple-action-client :init
                           (format nil "/~A/~A_gripper_controller/gripper_cmd" groupname tool)
                           (detachable-hand-ros-value "control_msgs::ParallelGripperCommand")
                           :groupname groupname))))
  (:detach-joint-name () (format nil "~A_detach_joint_0" side))
  (:effort-command-topic ()
    ;; ros_control JointGroupEffortController: command, ros2_control: commands
    (format nil "/~A/joint_group_effort_controller/~A" groupname
            (if (detachable-hand-ros2-p) "commands" "command")))
  (:joint-states-publish-topic () (format nil "/~A/joint_states" groupname))

  (:execution-mode () execution-mode)
  (:simulation-modep () (eq execution-mode :kinematic-simulator))

  (:wait-for-server (&optional timeout)
    (and action-client- (send action-client- :wait-for-server timeout)))

  (:use-kinematic-simulator ()
    (send self :start-kinematic-simulator))

  (:install-kinematic-simulator-timer ()
    (setq *top-selector-interval* 0.01)
    (setq *timer-job*
          (remove-if
           #'(lambda (job)
               (when (and (listp job) (>= (length job) 3))
                 (let ((body (caddr job)))
                   (and (listp body)
                        (eq (car body) 'send)
                        (eq (cadr body) self)
                        (eq (caddr body) :kinematic-simulator-callback)))))
           *timer-job*))
    (pushnew `(lambda () (send ,self :kinematic-simulator-callback))
             *timer-job*
             :test #'equal))

  (:start-kinematic-simulator ()
    (setq execution-mode :kinematic-simulator)
    (unless simulation-started-p
      (ros::ros-warn "Starting 'Kinematics Simulator' for ~A" side)
      (ros::advertise (send self :joint-states-publish-topic)
                      sensor_msgs::JointState 1)
      (send self :install-kinematic-simulator-timer)
      (send self :clear-hand-simulation current-hand-position)
      (send self :publish-joint-state)
      (setq simulation-started-p t))
    self)

  (:ensure-controller-or-simulator (&optional (timeout 1.0))
    (cond
      ((eq execution-mode :kinematic-simulator)
       nil)
      ((and action-client- (send action-client- :wait-for-server timeout))
       (setq execution-mode :controller)
       t)
      ((eq execution-mode :controller)
       (error "FollowJointTrajectory server is unavailable: ~A"
              controller-action))
      (t
       (send self :use-kinematic-simulator)
       (unless fallback-warned-p
         (ros::ros-warn
          "FollowJointTrajectory server is unavailable: ~A; using kinematic simulator"
          controller-action)
         (setq fallback-warned-p t))
       nil)))

  (:clear-hand-simulation (&optional previous)
    (setq simulation-target-sequence nil)
    (setq simulation-timer-sequence nil)
    (setq simulation-current-time 0.0)
    (setq simulation-previous-position
          (if previous (float previous) current-hand-position)))

  (:push-hand-simulation (position tm &optional previous)
    (when previous
      (setq simulation-previous-position (float previous)))
    (setq simulation-target-sequence
          (append simulation-target-sequence (list (float position))))
    (setq simulation-timer-sequence
          (append simulation-timer-sequence (list (max 0.0 (float tm)))))
    position)

  (:pop-hand-simulation ()
    (let ((target (car simulation-target-sequence))
          (tm (car simulation-timer-sequence))
          next-position)
      (when (and target tm)
        (unless simulation-previous-position
          (setq simulation-previous-position current-hand-position))
        (if (<= tm 0.0)
            (setq next-position target)
          (setq next-position
                (+ simulation-previous-position
                   (* (/ simulation-current-time tm)
                      (- target simulation-previous-position)))))
        (incf simulation-current-time 0.01)
        (when (or (<= tm 0.0) (>= simulation-current-time tm))
          (pop simulation-target-sequence)
          (pop simulation-timer-sequence)
          (setq next-position target)
          (setq simulation-previous-position next-position)
          (setq simulation-current-time 0.0))
        next-position)))

  (:publish-joint-state ()
    (let ((msg (instance sensor_msgs::JointState :init))
          (hand-joint (send self :hand-joint-name))
          (detach-joint (send self :detach-joint-name)))
      (detachable-hand-set-stamp-now (send msg :header))
      (send msg :name (list (format nil "~A_joint" side) detach-joint))
      (send msg :position
            (float-vector (float current-hand-position)
                          (float current-detach-position)))
      (send msg :velocity (float-vector 0.0 0.0))
      (send msg :effort (float-vector 0.0 0.0))
      (sethash hand-joint joint-states- current-hand-position)
      (sethash detach-joint joint-states- current-detach-position)
      (when (send self :simulation-modep)
        (ros::publish (send self :joint-states-publish-topic) msg))
      msg))

  (:queue-hand-simulation (position tm)
    (send self :clear-hand-simulation current-hand-position)
    (send self :push-hand-simulation position tm)
    (setq move-state- *detachable-hand-status-executing*)
    (send self :kinematic-simulator-callback)
    t)

  (:kinematic-simulator-callback ()
    (when (send self :simulation-modep)
      (let ((position (send self :pop-hand-simulation)))
        (when position
          (setq current-hand-position (float position))))
      (send self :publish-joint-state)
      (when (and move-state-
                 (= move-state- *detachable-hand-status-executing*)
                 (null simulation-timer-sequence))
        (setq move-state- *detachable-hand-status-succeeded*))))

  (:joint-states-callback (msg)
    (let ((names (send msg :name))
          (positions (send msg :position))
          (hand-joint (send self :hand-joint-name))
          (detach-joint (send self :detach-joint-name)))
      (dotimes (i (min (length names) (length positions)))
        (let ((name (elt names i))
              (position (elt positions i)))
          (sethash name joint-states- position)
          (cond
            ((string= name hand-joint)
             ;; ROS 2 tool joints are <side>_<tool>_joint; keep <side>_joint available
             (sethash (format nil "~A_joint" side) joint-states- position)
             (setq current-hand-position (float position)))
            ((string= name detach-joint)
             (setq current-detach-position (float position))))))))

  (:get-joint-state (joint-name)
    (send self :spin-once)
    (when (send self :simulation-modep)
      (send self :kinematic-simulator-callback))
    (gethash joint-name joint-states-))

  (:spin-once () (ros::spin-once groupname))

  ;; hand ------------------------------------------------------------------
  (:hand-effort () hand-effort)
  (:set-hand-effort (effort)
    "Set the grasp current limit [Nm] of the hand motor.  ROS 2 sends it as
the trajectory effort; ROS 1 position controllers ignore that, so it is
published on the hand joint of joint_group_effort_controller."
    (setq hand-effort (float effort))
    (when (and (not (detachable-hand-ros2-p))
               (not (send self :simulation-modep))
               (member (send self :hand-joint-name) (send self :effort-joints)
                       :test #'string=))
      (send self :set-detach-effort 0.0))
    hand-effort)

  (:move-hand (grasp-angle &key (wait t) (tm 1.0) (velocity 0.0) (acceleration 0.0) (effort 0.0))
    ;; ROS 1 ignores the trajectory effort as before; use :set-hand-effort there
    (if (not (send self :ensure-controller-or-simulator (if (detachable-hand-ros2-p) 5.0 1.0)))
        (progn
          (send self :queue-hand-simulation grasp-angle tm)
          (when wait
            (send self :wait-for-hand)))
      (let ((goal (if (detachable-hand-ros2-p)
                      (send self :gripper-goal grasp-angle effort)
                    (send self :trajectory-goal grasp-angle tm velocity acceleration effort))))
        (send action-client- :send-goal goal)
        (when wait
          (send action-client- :wait-for-result :timeout (+ tm 5.0))
          (setq move-state- (send action-client- :get-state))
          (unless (= move-state- *detachable-hand-status-succeeded*)
            (ros::ros-warn "Hand movement failed: ~A" move-state-)))))
    move-state-)

  (:trajectory-goal (grasp-angle tm velocity acceleration effort)
    "ROS 1: FollowJointTrajectory goal"
    (let ((goal (instance control_msgs::FollowJointTrajectoryGoal :init))
          (traj (instance trajectory_msgs::JointTrajectory :init))
          (point (instance trajectory_msgs::JointTrajectoryPoint :init)))
      (send traj :joint_names (list (send self :hand-joint-name)))
      (send point :positions (float-vector (float grasp-angle)))
      (send point :velocities (float-vector (float velocity)))
      (send point :accelerations (float-vector (float acceleration)))
      (send point :effort (float-vector (float (or effort 0.0))))
      (send point :time_from_start (detachable-hand-duration tm))
      (send traj :points (list point))
      (send goal :trajectory traj)
      goal))

  (:gripper-goal (grasp-angle effort)
    "ROS 2: ParallelGripperCommand goal, effort is the max effort (Goal Current)"
    (let ((goal (instance (detachable-hand-ros-value "control_msgs::ParallelGripperCommand_Goal") :init))
          (command (instance sensor_msgs::JointState :init)))
      (send command :name (list (send self :hand-joint-name)))
      (send command :position (float-vector (float grasp-angle)))
      (send command :effort (float-vector (float (or effort 0.0))))
      (send goal :command command)
      goal))

  (:cancel-move-hand ()
    (if (send self :simulation-modep)
        (progn
          (send self :clear-hand-simulation current-hand-position)
          (setq move-state- *detachable-hand-status-canceled*))
      (when action-client- (send action-client- :cancel-goal))))

  (:hand-moving-p ()
    (if (send self :simulation-modep)
        (not (null simulation-timer-sequence))
      (let ((state (and action-client- (send action-client- :get-state))))
        (and state (member state *detachable-hand-active-statuses* :test #'=)))))

  (:wait-for-hand ()
    (if (send self :simulation-modep)
        (progn
          (while (send self :hand-moving-p)
            (send self :kinematic-simulator-callback))
          (setq move-state- *detachable-hand-status-succeeded*)
          t)
      (progn
        (when action-client-
          (send action-client- :wait-for-result)
          (setq move-state- (send action-client- :get-state))))))

  (:open (&key (wait t) (tm (send self :gripper-open-time)) (velocity 0.0) (acceleration 0.0)
               (effort (send self :gripper-effort)))
    (send self :move-hand (send self :gripper-open-position)
          :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:close (&key (wait t) (tm 1.0) (velocity 0.0) (acceleration 0.0)
                (effort (send self :gripper-close-effort)))
    (send self :move-hand (send self :gripper-close-position)
          :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:move-named (state &key (tool "gripper") (wait t) (tm 1.0) (velocity 0.0) (acceleration 0.0) (effort 0.0))
    "move to the <group_state> state of tool in srdf/<side>.srdf"
    (send self :move-hand (send self :named-position tool state)
          :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:open-holder (&key (wait nil) (tm 1.0) (velocity 0.3) (acceleration 0.0) (effort 0.8))
    (send self :move-hand (send self :named-position "needle_holder" "open") :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:close-holder (&key (wait nil) (tm 1.0) (velocity 0.3) (acceleration 0.0) (effort 0.8))
    (send self :move-hand (send self :named-position "needle_holder" "close") :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:open-forceps (&key (wait nil) (tm 1.0) (velocity 1.0) (acceleration 0.0) (effort 0.0))
    (send self :move-hand (send self :named-position "forceps" "open") :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:close-forceps (&key (wait nil) (tm 1.0) (velocity 2.5) (acceleration 0.0) (effort 0.0))
    (send self :move-hand (send self :named-position "forceps" "close") :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:open-scissors (&key (wait nil) (tm 1.0) (velocity 3.0) (acceleration 0.0) (effort 0.9))
    (send self :move-hand (send self :named-position "forceps" "open_scissors") :wait nil :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:close-scissors (&key (wait nil) (tm 1.0) (velocity 2.0) (acceleration 0.0) (effort 0.8))
    (send self :move-hand (send self :named-position "forceps" "close_scissors") :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  ;; detacher ----------------------------------------------------------------
  (:effort-joints ()
    (let ((joints (ros::get-param
                   (format nil "/~A/joint_group_effort_controller/joints" groupname))))
      (if (and joints (listp joints))
          joints
        (list (send self :detach-joint-name)))))

  (:detach-effort-command (detach-effort)
    (let ((detach-joint (send self :detach-joint-name))
          (data nil))
      (dolist (joint (send self :effort-joints))
        (push (if (string= joint detach-joint)
                  (float detach-effort)
                (float hand-effort))
              data))
      (nreverse data)))

  (:wait-for-effort-subscriber (&optional (timeout 1.0))
    (let ((topic (send self :effort-command-topic))
          (start (ros::time-now)))
      (while (and (= (ros::get-num-subscribers topic) 0)
                  (< (send (ros::time- (ros::time-now) start) :to-sec) timeout))
        (ros::duration-sleep 0.05))
      (> (ros::get-num-subscribers topic) 0)))

  (:set-detach-effort (detach-effort)
    (let ((msg (instance std_msgs::Float64MultiArray :init)))
      (unless (send self :simulation-modep)
        (send self :wait-for-effort-subscriber))
      (send msg :data (apply #'float-vector (send self :detach-effort-command detach-effort)))
      (ros::publish (send self :effort-command-topic) msg)
      msg))

  (:stop-detach ()
    (send self :set-detach-effort 0.0))

  (:command-detach-current (current-ma &key (wait t) (tm 1.0))
    (let ((detach-effort (* (/ current-ma 1000.0) (send self :detach-torque-constant))))
      (send self :set-detach-effort detach-effort)
      (when (and tm (> tm 0.0))
        (ros::duration-sleep (float tm))
        (send self :stop-detach))))

  (:move-detach-joint (angle &key (wait t) (tm 1.0) (velocity 0.3) (acceleration 0.0) (effort 0.0))
    (send self :command-detach-current
          (/ (* effort 1000.0) (send self :detach-torque-constant)) :wait wait :tm tm))

  (:attach (&key (wait t) (tm 1.0) (velocity 0.3) (acceleration 0.0) effort)
    (send self :command-detach-current (- (send self :detach-current-ma)) :wait wait :tm tm))

  (:detach (&key (wait t) (tm 1.0) (velocity 0.3) (acceleration 0.0) effort)
    (send self :command-detach-current (send self :detach-current-ma) :wait wait :tm tm)))

(provide :detachable-hand-interface)
