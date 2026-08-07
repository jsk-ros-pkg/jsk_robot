(ros::load-ros-package "builtin_interfaces")
(ros::load-ros-package "control_msgs")
(ros::load-ros-package "geometry_msgs")
(ros::load-ros-package "sensor_msgs")
(ros::load-ros-package "trajectory_msgs")

(require :g1 "package://g1eus/g1.l")

(defun g1-shortest-angle (d0 d1)
  "Return the shortest angular displacement from D1 to D0 in degrees."
  (rad2deg (atan2 (sin (deg2rad (- d0 d1)))
                  (cos (deg2rad (- d0 d1))))))

(defun g1-angle-vector-copy (angles)
  (map float-vector #'(lambda (angle) angle) angles))

(defun g1-joint-list->joint-state (joint-list)
  (let ((msg (instance sensor_msgs::JointState :init)))
    (send msg :name (send-all joint-list :name))
    (send msg :position
          (map float-vector
               #'(lambda (joint)
                   (if (derivedp joint linear-joint)
                       (* 0.001 (send joint :joint-angle))
                     (deg2rad (send joint :joint-angle))))
               joint-list))
    (send msg :velocity (instantiate float-vector (length joint-list)))
    (send msg :effort (instantiate float-vector (length joint-list)))
    msg))

(defclass g1-controller-action-client
  :super ros::simple-action-client
  :slots (ri angle-vector-sequence timer-sequence current-time
          previous-angle-vector))

(defmethod g1-controller-action-client
  (:init (r &rest args)
    (setq ri r)
    (setq angle-vector-sequence nil)
    (setq timer-sequence nil)
    (setq current-time 0)
    (setq previous-angle-vector nil)
    (send-super* :init args))

  (:clear-angle-vector-simulation (&optional previous)
    (setq angle-vector-sequence nil)
    (setq timer-sequence nil)
    (setq current-time 0)
    (setq previous-angle-vector (if previous (copy-seq previous) nil)))

  (:push-angle-vector-simulation (av tm &optional previous)
    (when previous
      (setq previous-angle-vector (copy-seq previous)))
    (setq angle-vector-sequence
          (append angle-vector-sequence (list (copy-seq av))))
    (setq timer-sequence (append timer-sequence (list tm)))
    av)

  (:pop-angle-vector-simulation ()
    (let ((av (car angle-vector-sequence))
          (tm (car timer-sequence))
          scale-av next-av)
      (when (and av tm)
        (unless previous-angle-vector
          (setq previous-angle-vector (send ri :potentio-vector)))
        (setq scale-av (send ri :sub-angle-vector av previous-angle-vector))
        (setq next-av (v+ previous-angle-vector
                          (scale (/ current-time tm) scale-av)))
        (incf current-time 100.0)
        (when (>= current-time tm)
          (pop timer-sequence)
          (pop angle-vector-sequence)
          (setq next-av (v+ previous-angle-vector scale-av))
          (setq previous-angle-vector (copy-seq next-av))
          (setq current-time 0))
        next-av)))

  (:interpolatingp ()
    (if (send ri :simulation-modep)
        (not (null timer-sequence))
      (member (send self :get-state)
              (list ros::*action-goal-state-accepted*
                    ros::*action-goal-state-executing*
                    ros::*action-goal-state-canceling*)))))

(defclass g1-interface
  :super object
  :slots (robot upper-body-action action-client hand-actions-
          hand-joint-names- joint-states-topic pub-joint-states-topic
          cmd-vel-topic current-angle-vector execution-mode fallback-warned-p
          objects viewer simulation-default-look-all-p simulation-started-p
          groupname))

(defmethod g1-interface
  (:init (&key ((:robot r) g1-robot)
               ((:upper-body-action uba)
                "/upper_body_controller/follow_joint_trajectory")
               ((:joint-states-topic jst) "/joint_states")
               ((:publish-joint-states-topic pjst) nil)
               ((:cmd-vel-topic cvt) "/cmd_vel")
               ((:objects objs) nil)
               ((:execution-mode em) :auto)
               ((:simulation-look-all sim-look-all) t)
               ((:hand-controller-timeout hct) 3.0)
               ((:groupname gp) nil))
    (unless (ros::ok)
      (ros::roseus "g1_interface"))
    (setq robot (if (derivedp r metaclass) (instance r :init) r))
    (setq upper-body-action uba)
    (setq joint-states-topic jst)
    (setq pub-joint-states-topic pjst)
    (setq cmd-vel-topic cvt)
    (setq current-angle-vector (copy-seq (send robot :angle-vector)))
    (setq execution-mode em)
    (setq fallback-warned-p nil)
    (setq objects objs)
    (setq viewer nil)
    (setq simulation-default-look-all-p sim-look-all)
    (setq simulation-started-p nil)
    (setq groupname gp)
    (setq action-client
          (instance g1-controller-action-client :init self upper-body-action
                    control_msgs::FollowJointTrajectory
                    :groupname groupname))
    (setq hand-actions- (make-hash-table))
    (setq hand-joint-names- (make-hash-table))
    (ros::subscribe joint-states-topic sensor_msgs::JointState
                    #'send self :joint-state-callback 1)
    (ros::advertise cmd-vel-topic geometry_msgs::Twist 1)
    (send self :init-hand-actions hct)
    (when (eq execution-mode :kinematic-simulator)
      (send self :start-kinematic-simulator))
    self)

  (:robot () robot)
  (:objects (&optional objs)
    (when objs
      (setq objects objs)
      (when viewer
        (send viewer :objects (cons robot objects))
        (send self :draw-objects)))
    objects)

  ;; Compatibility with the commonly used robot-interface API.
  (:state () self)
  (:potentio-vector () (copy-seq current-angle-vector))
  (:action-client () action-client)
  (:execution-mode () execution-mode)
  (:simulation-modep () (eq execution-mode :kinematic-simulator))
  (:spin-once () (if groupname (ros::spin-once groupname) (ros::spin-once)))

  (:wait-for-server (&optional timeout)
    (send action-client :wait-for-server timeout))

  (:use-kinematic-simulator ()
    (send self :start-kinematic-simulator))

  (:joint-states-publish-topic ()
    (or pub-joint-states-topic joint-states-topic))

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
    (setq current-angle-vector (copy-seq (send robot :angle-vector)))
    (unless simulation-started-p
      (ros::ros-warn "Starting 'Kinematics Simulator' for ~A"
                     (send robot :name))
      (let ((old-viewer user::*viewer*))
        (when (and x::*display* (> x::*display* 0))
          (setq viewer (get (geo::find-viewer (send robot :name)) :pickviewer))
          (unless viewer
            (setq viewer
                  (instance x::irtviewer :create
                            :title (format nil "~A Kinematics Simulator"
                                           (send robot :name))
                            :view-name (send robot :name)
                            :draw-floor t)))
          (send viewer :objects (cons robot objects))
          (send self :draw-objects)
          (when old-viewer
            (setq user::*viewer* old-viewer))))
      (ros::advertise (send self :joint-states-publish-topic)
                      sensor_msgs::JointState)
      (send self :install-kinematic-simulator-timer)
      (send action-client :clear-angle-vector-simulation current-angle-vector)
      (send self :publish-joint-state)
      (setq simulation-started-p t))
    self)

  (:ensure-controller-or-simulator (&optional (timeout 10.0))
    (cond
      ((eq execution-mode :kinematic-simulator)
       nil)
      ((send action-client :wait-for-server timeout)
       (setq execution-mode :controller)
       t)
      ((eq execution-mode :controller)
       (error "FollowJointTrajectory server is unavailable: ~A"
              upper-body-action))
      (t
       (send self :use-kinematic-simulator)
       (unless fallback-warned-p
         (ros::ros-warn
          "FollowJointTrajectory server is unavailable: ~A; using kinematic simulator"
          upper-body-action)
         (setq fallback-warned-p t))
       nil)))

  (:draw-objects (&key look-all)
    (when viewer
      (send viewer :objects (cons robot objects))
      (when (or simulation-default-look-all-p look-all)
        (send viewer :look-all
              (send (geo::make-bounding-box
                     (flatten (send-all (x::draw-things robot) :vertices)))
                    :grow 0.3)))
      (send viewer :draw-objects)))

  (:publish-joint-state (&optional (joint-list (send robot :joint-list)))
    (let ((msg (g1-joint-list->joint-state joint-list))
          (now (ros::time-now)))
      (send msg :header :stamp :sec (send now :sec))
      (send msg :header :stamp :nanosec (send now :nsec))
      (when (send self :simulation-modep)
        (ros::publish (send self :joint-states-publish-topic) msg))
      msg))

  (:queue-angle-vector-simulation (sequence times)
    (send action-client :clear-angle-vector-simulation current-angle-vector)
    (mapc #'(lambda (angles tm)
              (send action-client :push-angle-vector-simulation angles tm))
          sequence times)
    (send self :kinematic-simulator-callback)
    t)

  (:kinematic-simulator-callback ()
    (when (send self :simulation-modep)
      (let ((av (send action-client :pop-angle-vector-simulation)))
        (when av
          (send robot :angle-vector av)
          (setq current-angle-vector (copy-seq av))))
      (send self :publish-joint-state)
      (send self :draw-objects)))

  (:joint-names ()
    (mapcar #'(lambda (joint) (send joint :name)) (send robot :joint-list)))

  (:upper-body-joint-names ()
    (list
     "waist_yaw_joint"
     "waist_roll_joint"
     "waist_pitch_joint"
     "left_shoulder_pitch_joint"
     "left_shoulder_roll_joint"
     "left_shoulder_yaw_joint"
     "left_elbow_joint"
     "left_wrist_roll_joint"
     "left_wrist_pitch_joint"
     "left_wrist_yaw_joint"
     "right_shoulder_pitch_joint"
     "right_shoulder_roll_joint"
     "right_shoulder_yaw_joint"
     "right_elbow_joint"
     "right_wrist_roll_joint"
     "right_wrist_pitch_joint"
     "right_wrist_yaw_joint"))

  (:upper-body-controller ()
    (list
     (list
      (cons :controller-action upper-body-action)
      (cons :action-type control_msgs::FollowJointTrajectory)
      (cons :joint-names (send self :upper-body-joint-names)))))

  (:default-controller ()
    (send self :upper-body-controller))

  (:joint-index (joint-name)
    (position joint-name (send self :joint-names) :test #'string=))

  (:upper-body-angle-vector (full-angle-vector)
    (map float-vector
         #'(lambda (joint-name)
             (let ((index (send self :joint-index joint-name)))
               (unless index
                 (error "joint ~A is not found in g1 model" joint-name))
               (elt full-angle-vector index)))
         (send self :upper-body-joint-names)))

  (:normalize-angle-vector (angles)
    (let ((full-count (length (send self :joint-names)))
          (upper-count (length (send self :upper-body-joint-names)))
          full index)
      (cond
        ((= (length angles) full-count)
         (g1-angle-vector-copy angles))
        ((= (length angles) upper-count)
         (setq full (copy-seq current-angle-vector))
         (mapc #'(lambda (joint-name angle)
                   (setq index (send self :joint-index joint-name))
                   (unless index
                     (error "joint ~A is not found in g1 model" joint-name))
                   (setf (elt full index) angle))
               (send self :upper-body-joint-names)
               (coerce angles cons))
         full)
        (t
         (error "expected ~A full-body angles or ~A upper-body angles, got ~A"
                full-count upper-count (length angles))))))

  (:make-duration (tm)
    (unless (and (numberp tm) (> tm 0))
      (error "trajectory duration must be positive milliseconds: ~A" tm))
    (let* ((nanoseconds (round (* tm 1000000)))
           (duration (instance builtin_interfaces::Duration :init)))
      (send duration :sec (floor (/ nanoseconds 1000000000)))
      (send duration :nanosec (mod nanoseconds 1000000000))
      duration))

  (:make-point (angles tm)
    (let ((point (instance trajectory_msgs::JointTrajectoryPoint :init))
          (zeros (instantiate float-vector (length angles))))
      (send point :positions
            (map float-vector #'(lambda (degree) (deg2rad degree)) angles))
      (send point :velocities zeros)
      (send point :time_from_start (send self :make-duration tm))
      point))

  (:angle-vector (angles &optional (tm 1000))
    "Send ANGLES (degrees) to the ROS 2 upper-body trajectory action."
    (send self :angle-vector-sequence (list angles) (list tm)))

  (:angle-vector-sequence (sequence &optional times)
    "Send a degree angle-vector sequence. TIMES contains segment msec."
    (unless times (setq times (make-list (length sequence) :initial-element 1000)))
    (unless (= (length sequence) (length times))
      (error "angle-vector-sequence and times must have the same length"))
    (when (null sequence)
      (error "angle-vector-sequence must not be empty"))
    (dolist (tm times)
      (unless (and (numberp tm) (> tm 0))
        (error "trajectory duration must be positive milliseconds: ~A" tm)))
    (let ((full-sequence
           (mapcar #'(lambda (angles)
                       (send self :normalize-angle-vector angles))
                   sequence)))
      (if (not (send self :ensure-controller-or-simulator 10.0))
          (send self :queue-angle-vector-simulation full-sequence times)
        (let ((trajectory (instance trajectory_msgs::JointTrajectory :init))
              (elapsed 0)
              points)
          (send trajectory :joint_names (send self :upper-body-joint-names))
          (mapc #'(lambda (angles tm)
                    (incf elapsed tm)
                    (setq points
                          (append points
                                  (list
                                   (send self :make-point
                                         (send self :upper-body-angle-vector angles)
                                         elapsed)))))
                full-sequence times)
          (send trajectory :points points)
          (let ((goal (instance control_msgs::FollowJointTrajectory_Goal :init)))
            (send goal :trajectory trajectory)
            (send action-client :send-goal goal))))))

  (:sub-angle-vector (angles reference)
    (map float-vector #'g1-shortest-angle angles reference))

  (:joint-state-callback (msg)
    (let ((names (send msg :name))
          (positions (send msg :position))
          (joints (send robot :joint-list))
          measured)
      (setq measured
            (map float-vector
                 #'(lambda (joint old-angle)
                     (let ((index (position (send joint :name) names
                                            :test #'string=)))
                       (if index (rad2deg (elt positions index)) old-angle)))
                 joints current-angle-vector))
      (let ((i 0))
        (dolist (joint joints)
          (let ((index (position (send joint :name) names :test #'string=)))
            (when index
              (send joint :joint-angle
                    (max (send joint :min-angle)
                         (min (elt measured i) (send joint :max-angle))))))
          (incf i)))
      (setq current-angle-vector measured)))

  (:interpolatingp ()
    (if (eq execution-mode :kinematic-simulator)
        (send action-client :interpolatingp)
      (progn
        (send action-client :spin-once)
        (send action-client :interpolatingp))))

  (:wait-interpolation ()
    (when (eq execution-mode :kinematic-simulator)
      (while (send self :interpolatingp)
        (send self :kinematic-simulator-callback))
      (return-from :wait-interpolation t))
    (unless (send action-client :wait-for-result)
      (return-from :wait-interpolation nil))
    (let ((result (send action-client :get-result)))
      (and result
           (= (send action-client :get-state)
              ros::*action-goal-state-succeeded*)
           (= (send result :error_code) 0))))

  (:arm-to-hand-controller-name (arm)
    (case arm
      (:rarm "right_hand_controller")
      (:larm "left_hand_controller")
      (t nil)))

  (:default-hand-joint-names (arm)
    (case arm
      (:rarm
       (list
        "R_thumb_proximal_yaw_joint"
        "R_thumb_proximal_pitch_joint"
        "R_index_proximal_joint"
        "R_middle_proximal_joint"
        "R_ring_proximal_joint"
        "R_pinky_proximal_joint"))
      (:larm
       (list
        "L_thumb_proximal_yaw_joint"
        "L_thumb_proximal_pitch_joint"
        "L_index_proximal_joint"
        "L_middle_proximal_joint"
        "L_ring_proximal_joint"
        "L_pinky_proximal_joint"))
      (t nil)))

  (:init-hand-actions (&optional (timeout 3.0))
    (dolist (arm (list :rarm :larm))
      (let* ((controller-name (send self :arm-to-hand-controller-name arm))
             (action-name (format nil "/~A/follow_joint_trajectory"
                                  controller-name))
             (joint-names (ros::get-param (format nil "~A/joints" controller-name)
                                           (send self :default-hand-joint-names arm)))
             action)
        (sethash arm hand-joint-names- joint-names)
        (unless (send self :simulation-modep)
          (setq action
                (instance ros::simple-action-client :init action-name
                          control_msgs::FollowJointTrajectory
                          :groupname groupname))
          (if (send action :wait-for-server timeout)
              (sethash arm hand-actions- action)
            (ros::ros-warn "~A is not responding" action-name)))))
    hand-actions-)

  (:hand-action (arm)
    (gethash arm hand-actions-))

  (:hand-angle-vector (arm av tm &key (wait t))
    (let ((action (gethash arm hand-actions-))
          (names (gethash arm hand-joint-names-)))
      (cond
        ((null action)
         (ros::ros-warn "hand action for ~A is not available" arm)
         nil)
        ((null names)
         (ros::ros-warn "joint names for ~A hand are not available" arm)
         nil)
        ((/= (length av) (length names))
         (ros::ros-warn
          "hand angle-vector length ~A does not match joint names length ~A for ~A"
          (length av) (length names) arm)
         nil)
        (t
         (let ((goal (instance control_msgs::FollowJointTrajectory_Goal :init))
               (trajectory (instance trajectory_msgs::JointTrajectory :init))
               (result nil))
           (send trajectory :joint_names names)
           (send trajectory :points
                 (list (send self :make-point av tm)))
           (send goal :trajectory trajectory)
           (send action :send-goal goal)
           (when wait
             (send action :wait-for-result)
             (setq result (send action :get-result))
             (when (and result (/= (send result :error_code) 0))
               (warning-message
                3
                ":hand-angle-vector error code ~A returns from JTA: ~A~%"
                (send result :error_code) (send result :error_string))))
           result)))))

  (:cancel-hand-angle-vector (arm)
    (let ((action (gethash arm hand-actions-)))
      (if action
          (send action :cancel-goal)
        (ros::ros-warn "hand action for ~A is not available" arm))))

  (:hand-interpolatingp (arm)
    (send self :spin-once)
    (let ((action (gethash arm hand-actions-)))
      (and action
           (member (send action :get-state)
                   (list ros::*action-goal-state-accepted*
                         ros::*action-goal-state-executing*
                         ros::*action-goal-state-canceling*)))))

  (:stop-grasp (arm)
    (send self :hand-angle-vector arm #f(0 0 0 0 0 0) 1000))

  (:prepare-grasp (arm)
    (send self :hand-angle-vector arm #f(100 0 0 0 0 0) 1000))

  (:start-grasp (arm)
    (send self :hand-angle-vector arm #f(40 20 80 80 80 80) 1000))

  (:default-grasp (arm)
    (send self :hand-angle-vector arm #f(10 30 100 100 100 100) 1000))

  (:cmd-vel-topic (&optional topic)
    (when topic
      (setq cmd-vel-topic topic)
      (ros::advertise cmd-vel-topic geometry_msgs::Twist 1))
    cmd-vel-topic)

  (:cmd-vel (&optional (x 0.0) (y 0.0) (yaw 0.0))
    (let ((msg (instance geometry_msgs::Twist :init)))
      (send msg :linear :x x)
      (send msg :linear :y y)
      (send msg :angular :z yaw)
      (ros::publish cmd-vel-topic msg)
      msg))

  (:go-velocity (x y yaw)
    (send self :cmd-vel x y yaw))

  (:stop-base ()
    (send self :cmd-vel 0.0 0.0 0.0)))

(defun g1-init
  (&key ((:upper-body-action uba)
         "/upper_body_controller/follow_joint_trajectory")
        ((:joint-states-topic jst) "/joint_states")
        ((:publish-joint-states-topic pjst) nil)
        ((:cmd-vel-topic cvt) "/cmd_vel")
        ((:objects objs) nil)
        ((:execution-mode em) :auto)
        ((:simulation-look-all sim-look-all) t)
        ((:hand-controller-timeout hct) 3.0)
        ((:groupname gp) nil))
  (unless (ros::ok)
    (ros::roseus "g1_interface"))
  (when (or (not (boundp '*g1*)) (null *g1*))
    (setq *g1* (instance g1-robot :init)))
  (setq *ri* (instance g1-interface :init
                       :robot *g1*
                       :upper-body-action uba
                       :joint-states-topic jst
                       :publish-joint-states-topic pjst
                       :cmd-vel-topic cvt
                       :objects objs
                       :execution-mode em
                       :simulation-look-all sim-look-all
                       :hand-controller-timeout hct
                       :groupname gp))
  *ri*)
