#!/usr/bin/env roseus

(require "package://roseus/euslisp/actionlib2.l")
(unless (ros::ok)
  (ros::roseus "rhand_action_controller"))
(ros::load-ros-package "action_msgs")
(ros::load-ros-package "builtin_interfaces")
(ros::load-ros-package "control_msgs")
(ros::load-ros-package "sensor_msgs")
(ros::load-ros-package "std_msgs")
(ros::load-ros-package "trajectory_msgs")

(defparameter *rhand-detach-torque-constant* 2.0)
(defparameter *rhand-detach-current-ma* 500.0)
(defparameter *rhand-effort-limit* 0.50)
(defparameter *rhand-gripper-effort* 0.30)
(defparameter *rhand-gripper-close-effort* 0.50)
(defparameter *rhand-gripper-close-position* -1.5)
(defparameter *rhand-gripper-open-time* 0.25)

(defclass rhand-interface
  :super propertied-object
  :slots (action-client- move-state- joint-states- groupname controller-action
          execution-mode fallback-warned-p simulation-started-p
          current-hand-position current-detach-position
          simulation-target-sequence simulation-timer-sequence
          simulation-current-time simulation-previous-position))

(defmethod rhand-interface
  (:init (&key ((:groupname grp) "rhand")
               ((:execution-mode em) :auto)
               ((:server-timeout server-timeout) 1.0))
    (setq groupname grp)
    (setq controller-action
          "/rhand/position_joint_trajectory_controller/follow_joint_trajectory")
    (setq execution-mode em)
    (setq fallback-warned-p nil)
    (setq simulation-started-p nil)
    (setq current-hand-position 0.0)
    (setq current-detach-position 0.0)
    (setq simulation-target-sequence nil)
    (setq simulation-timer-sequence nil)
    (setq simulation-current-time 0.0)
    (setq simulation-previous-position nil)
    (setq move-state- action_msgs::GoalStatus::*STATUS_SUCCEEDED*)
    (setq joint-states- (make-hash-table :test #'equal))
    (when groupname
      (ros::create-nodehandle groupname))
    (setq action-client-
          (instance ros::simple-action-client :init
                    controller-action
                    control_msgs::FollowJointTrajectory
                    :groupname groupname))
    (ros::advertise "/rhand/joint_group_effort_controller/commands" std_msgs::Float64MultiArray 1)
    (ros::subscribe "/rhand/joint_states" sensor_msgs::JointState
                    #'send self :joint-states-callback 1 :groupname groupname)
    (when (eq execution-mode :kinematic-simulator)
      (send self :start-kinematic-simulator))
    (when (eq execution-mode :auto)
      (send self :ensure-controller-or-simulator server-timeout))
    self)

  (:execution-mode () execution-mode)
  (:simulation-modep () (eq execution-mode :kinematic-simulator))

  (:wait-for-server (&optional timeout)
    (send action-client- :wait-for-server timeout))

  (:use-kinematic-simulator ()
    (send self :start-kinematic-simulator))

  (:joint-states-publish-topic () "/rhand/joint_states")

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
      (ros::ros-warn "Starting 'Kinematics Simulator' for rhand")
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
      ((send action-client- :wait-for-server timeout)
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
          (now (ros::time-now)))
      (send msg :header :stamp :sec (send now :sec))
      (send msg :header :stamp :nanosec (send now :nsec))
      (send msg :name (list "rhand_joint" "rhand_detach_joint_0"))
      (send msg :position
            (float-vector (float current-hand-position)
                          (float current-detach-position)))
      (send msg :velocity (float-vector 0.0 0.0))
      (send msg :effort (float-vector 0.0 0.0))
      (sethash "rhand_joint" joint-states- current-hand-position)
      (sethash "rhand_detach_joint_0" joint-states- current-detach-position)
      (when (send self :simulation-modep)
        (ros::publish (send self :joint-states-publish-topic) msg))
      msg))

  (:queue-hand-simulation (position tm)
    (send self :clear-hand-simulation current-hand-position)
    (send self :push-hand-simulation position tm)
    (setq move-state- action_msgs::GoalStatus::*STATUS_EXECUTING*)
    (send self :kinematic-simulator-callback)
    t)

  (:kinematic-simulator-callback ()
    (when (send self :simulation-modep)
      (let ((position (send self :pop-hand-simulation)))
        (when position
          (setq current-hand-position (float position))))
      (send self :publish-joint-state)
      (when (and move-state-
                 (= move-state- action_msgs::GoalStatus::*STATUS_EXECUTING*)
                 (null simulation-timer-sequence))
        (setq move-state- action_msgs::GoalStatus::*STATUS_SUCCEEDED*))))

  (:joint-states-callback (msg)
    (let ((names (send msg :name))
          (positions (send msg :position)))
      (dotimes (i (length names))
        (let ((name (elt names i))
              (position (elt positions i)))
          (sethash name joint-states- position)
          (cond
            ((string= name "rhand_joint")
             (setq current-hand-position (float position)))
            ((string= name "rhand_detach_joint_0")
             (setq current-detach-position (float position))))))))

  (:get-joint-state (joint-name)
    (send self :spin-once)
    (when (send self :simulation-modep)
      (send self :kinematic-simulator-callback))
    (gethash joint-name joint-states-))

  (:spin-once () (ros::spin-once groupname))

  (:move-hand (grasp-angle &key (wait t) (tm 1.0) (velocity 0.0) (acceleration 0.0) (effort 0.0))
    (if (not (send self :ensure-controller-or-simulator 1.0))
        (progn
          (send self :queue-hand-simulation grasp-angle tm)
          (when wait
            (send self :wait-for-hand)))
      (let* ((tm-float (float tm))
             (tm-sec (floor tm-float))
             (tm-nsec (floor (* (- tm-float tm-sec) 1000000000)))
             (goal (instance control_msgs::FollowJointTrajectory_Goal :init))
             (traj (instance trajectory_msgs::JointTrajectory :init))
             (point (instance trajectory_msgs::JointTrajectoryPoint :init)))
        (send traj :joint_names (list "rhand_joint"))
        (send point :positions (float-vector (float grasp-angle)))
        (send point :velocities (float-vector (float velocity)))
        (send point :accelerations (float-vector (float acceleration)))
        (send point :effort (float-vector (float effort)))
        (send point :time_from_start
              (instance builtin_interfaces::Duration :init :sec tm-sec :nanosec tm-nsec))
        (send traj :points (list point))
        (send goal :trajectory traj)
        (send action-client- :send-goal goal)
        (when wait
          (send action-client- :wait-for-result :timeout (+ tm 5.0))
          (setq move-state- (send action-client- :get-state))
          (unless (= move-state- action_msgs::GoalStatus::*STATUS_SUCCEEDED*)
            (ros::ros-warn "Hand movement failed: ~A" move-state-)))))
    move-state-)

  (:move-detach-joint (angle &key (wait t) (tm 1.0) (velocity 0.3) (acceleration 0.0) (effort 0.0))
    (send self :command-detach-current (/ (* effort 1000.0) *rhand-detach-torque-constant*) :wait wait :tm tm))

  (:detach-effort-command (detach-effort)
    (let ((joints (ros::get-param "/rhand/joint_group_effort_controller/joints" (list "rhand_detach_joint_0")))
          (data nil))
      (dolist (joint joints)
        (push (if (string= joint "rhand_detach_joint_0")
                  (float detach-effort)
                (float *rhand-effort-limit*))
              data))
      (nreverse data)))

  (:set-detach-effort (detach-effort)
    (let ((msg (instance std_msgs::Float64MultiArray :init)))
      (send msg :data (apply #'float-vector (send self :detach-effort-command detach-effort)))
      (ros::publish "/rhand/joint_group_effort_controller/commands" msg)))

  (:stop-detach ()
    (send self :set-detach-effort 0.0))

  (:command-detach-current (current-ma &key (wait t) (tm 1.0))
    (let ((detach-effort (* (/ current-ma 1000.0) *rhand-detach-torque-constant*)))
      (send self :set-detach-effort detach-effort)
      (when (and tm (> tm 0.0))
        (ros::duration-sleep tm)
        (send self :stop-detach))))

  (:cancel-move-hand ()
    (if (send self :simulation-modep)
        (progn
          (send self :clear-hand-simulation current-hand-position)
          (setq move-state- action_msgs::GoalStatus::*STATUS_CANCELED*))
      (send action-client- :cancel-goal)))
  (:hand-moving-p ()
    (if (send self :simulation-modep)
        (not (null simulation-timer-sequence))
      (let ((state (send action-client- :get-state)))
        (or (= state action_msgs::GoalStatus::*STATUS_ACCEPTED*)
            (= state action_msgs::GoalStatus::*STATUS_EXECUTING*)
            (= state action_msgs::GoalStatus::*STATUS_CANCELING*)))))

  (:open (&key (wait t) (tm *rhand-gripper-open-time*) (velocity 0.0) (acceleration 0.0) (effort *rhand-gripper-effort*))
    (send self :move-hand 0.0 :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:close (&key (wait t) (tm 1.0) (velocity 0.0) (acceleration 0.0) (effort *rhand-gripper-close-effort*))
    (send self :move-hand *rhand-gripper-close-position* :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:open-holder (&key (wait nil) (tm 1.0) (velocity 0.3) (acceleration 0.0) (effort 0.8))
    (send self :move-hand -1.87 :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:close-holder (&key (wait nil) (tm 1.0) (velocity 0.3) (acceleration 0.0) (effort 0.8))
    (send self :move-hand 1.0 :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:open-forceps (&key (wait nil) (tm 1.0) (velocity 1.0) (acceleration 0.0) (effort 0.0))
    (send self :move-hand -2.2 :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:close-forceps (&key (wait nil) (tm 1.0) (velocity 2.5) (acceleration 0.0) (effort 0.0))
    (send self :move-hand -3.0 :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:open-scissors (&key (wait nil) (tm 1.0) (velocity 3.0) (acceleration 0.0) (effort 0.9))
    (send self :move-hand -0.8 :wait nil :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:close-scissors (&key (wait nil) (tm 1.0) (velocity 2.0) (acceleration 0.0) (effort 0.8))
    (send self :move-hand -0.1 :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:attach (&key (wait t) (tm 1.0) (velocity 0.3) (acceleration 0.0) effort)
    (send self :command-detach-current (- *rhand-detach-current-ma*) :wait wait :tm tm))

  (:detach (&key (wait t) (tm 1.0) (velocity 0.3) (acceleration 0.0) effort)
    (send self :command-detach-current *rhand-detach-current-ma* :wait wait :tm tm))

  (:wait-for-hand ()
    (if (send self :simulation-modep)
        (progn
          (while (send self :hand-moving-p)
            (send self :kinematic-simulator-callback))
          (setq move-state- action_msgs::GoalStatus::*STATUS_SUCCEEDED*)
          t)
      (progn
        (send action-client- :wait-for-result)
        (setq move-state- (send action-client- :get-state))))))

(defun rhand-init ()
  (if (not (boundp '*rhand*))
      (setq *rhand* (instance rhand-interface :init)))
  *rhand*)

(defun rhand (&key grasp (velocity 0.0) (acceleration 0.0) (effort 0.0))
  (unless (boundp '*rhand*) (rhand-init))
  (let ((grasp-angle
         (cond ((eq grasp 'open) 0.0)
               ((eq grasp 'close) -3.0)
               ((numberp grasp) (float grasp))
               (t 0.0))))
    (send *rhand* :move-hand grasp-angle :velocity velocity :acceleration acceleration :effort effort)))
