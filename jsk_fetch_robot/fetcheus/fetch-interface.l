(require "package://fetcheus/fetch-utils.l")
(require "package://pr2eus/robot-interface.l")
(require "package://pr2eus_moveit/euslisp/robot-moveit.l")

(ros::load-ros-package "fetcheus")
(ros::load-ros-package "fetch_driver_msgs")
(ros::load-ros-package "robot_controllers_msgs")

(defclass fetch-interface
  :super robot-move-base-interface
  :slots (gripper-action moveit-robot fetch-controller-action)
  )

(defmethod fetch-interface
  (:init (&key (default-collision-object t) &rest args)
   (prog1 (send-super* :init :robot fetch-robot :base-frame-id "base_link" :odom-topic "/odom_combined" :base-controller-action-name nil args)
     (send self :add-controller :arm-controller)
     (send self :add-controller :torso-controller)
     (send self :add-controller :head-controller)
     ;; check if fetch_driver_msgs exists, fetch_ros in released on kinetic/melodic, but fetch_msgs is not released yet.
     (if (boundp 'fetch_driver_msgs::GripperState)
	 (ros::subscribe "gripper_state" fetch_driver_msgs::GripperState #'send self :fetch-gripper-state-callback :groupname groupname)
       (ros::ros-warn "Could not find fetch_driver_msgs package, please install from source https://github.com/fetchrobotics/fetch_msgs"))
     (setq gripper-action
           (instance ros::simple-action-client :init
                     "/gripper_controller/gripper_action"
                     control_msgs::GripperCommandAction
                     :groupname groupname))
     (setq moveit-robot (instance fetch-robot :init))
     (send self :set-moveit-environment (instance fetch-moveit-environment :init :robot moveit-robot))
     (when (and (boundp '*co*) default-collision-object)
       (send self :delete-headbox-collision-object)
       (send self :delete-keepout-collision-object)
       (send self :delete-ground-collision-object)
       (send self :add-headbox-collision-object)
       (send self :add-keepout-collision-object)
       (send self :add-ground-collision-object))
     (setq fetch-controller-action
           (instance ros::simple-action-client :init
                     "/query_controller_states"
                     robot_controllers_msgs::QueryControllerStatesAction))
     ))
  (:state (&rest args)
   "We do not have :wait-until-update option for :state :worldcoords.
In other cases, :state calls with :wait-until-update by default, since Fetch publishes /joint_states from body and gripper at almost same frequency.
Example usage:
(send *ri* :state :worldcoords frame_id) returns current coordinates in frame_id frame. If frame_id is not given, it returns current coordinates in '/world' frame.
(send *ri* :state :potentio-vector) returns current real fetch's angle vector.
"
   (case (car args)
     (:worldcoords
       (send-super :state :worldcoords (or (cadr args) "world")))
     (t
       (send-super* :state
                    (if (member :wait-until-update args)
                        args (append args (list :wait-until-update t)))))))
  (:check-continuous-joint-move-over-180 ;; can be removed if http//github.com/jsk-ros-pkg/jsk_pr2eus/pull/322 merged
   (diff-av)
   (let ((i 0) add-new-trajectory-point)
     (dolist (j (send robot :joint-list))
       ;; for continuous rotational joint
       (when (and (> (- (send j :max-angle) (send j :min-angle)) 360)
                  (> (abs (elt diff-av i)) 180))
         (ros::ros-warn "continuous joint (~A) moves ~A degree, commanded joint differs from original trajectory to avoid unintentional 360 rotation" (send j :name) (elt diff-av i))
         (setq add-new-trajectory-point t))
       (incf i (send j :joint-dof)))
     add-new-trajectory-point))
  (:angle-vector-raw (av &optional (tm 3000) &rest args)
   (let* ((prev-av (send self :state :potentio-vector :wait-until-update t))
          (diff-av (v- av prev-av)))
     (when (send self :check-continuous-joint-move-over-180 diff-av)
       (let* (avs (minjerk (instance minjerk-interpolator :init))
                  (scale-av (send self :sub-angle-vector av prev-av))
                  dist div)
         (setq tm (float tm))
         (setq dist (abs (geo::find-extream (coerce diff-av cons) #'abs #'>=)))
         ;; Use small interpolation time
         ;; https://github.com/euslisp/jskeus/pull/596#issuecomment-765213357
         (setq div (* 10 (round (/ dist 120.0))))
         (send minjerk :reset
               :position-list (list prev-av (v+ prev-av scale-av))
               :time-list (list tm))
         (send minjerk :start-interpolation)
         (send minjerk :pass-time (/ tm div))
         (dotimes (i div)
           (setq avs (append avs (list (send minjerk :pass-time (/ tm div))))))
         (send* self :angle-vector-sequence-raw avs (make-list div :initial-element (/ tm div)) args)
         (return-from :angle-vector-raw (car (last avs)))))
     (send-super* :angle-vector av tm args)))
  (:angle-vector-sequence-raw (&rest args) (send-super* :angle-vector-sequence args))
  (:angle-vector
   (av &optional (tm 3000) &rest args) ;; (ctype controller-type) (start-time 0) &rest args
                                       ;;  &key (use-torso t) (clear-velocities t) &allow-other-keys)
   "Send joind angle to robot with self-collision motion planning, this method retuns immediately, so use :wait-interpolation to block until the motion stops.
- av : joint angle vector [rad]
- tm : time to goal in [msec]
- use-torso : set t to use torso
"
   (let ((ctype controller-type) (start-time 0) (start-offset-time nil) (use-torso t) (clear-velocities t))
   ;; as of 0.3.x, :angle-vector (robot-interface) :acceps tm ctype start-time as optional arguments, but in here we prefer old API
   (if (= (length args) 1) ;; args must be ctype
       (setq ctype (car args)
             args (cdr args)))
   (if (and (>= (length args) 2) (null (member (car args) '(:use-torso :start-time :clear-velocities))));; args must be ctype start-time
       (setq ctype (car args)
             start-time (cadr args)
             args (cddr args)))
   (if (member :use-torso args) (setq use-torso (cadr (member :use-torso args))))
   (if (member :start-time args) (setq start-time (cadr (member :start-time args))))
   (if (member :start-offset-time args) (setq start-offset-time (cadr (member :start-offset-time args))))
   (if (member :clear-velocities args) (setq clear-velocities (cadr (member :clear-velocities args))))
   ;; for simulation mode
   (when (and (not (boundp '*co*)) (send self :simulation-modep))
     (if (member :use-torso args)
       (setq args (append (subseq args 0 (position :use-torso args))
                          (if (> (length args) (+ 2 (position :use-torso args)))
                            (subseq args (+ (position :use-torso args) 2))))))
     (return-from :angle-vector (send* self :angle-vector-raw av tm ctype start-time args)))
   ;;
   (when (not (numberp tm))
     (ros::warn ":angle-vector tm is not a number, use :angle-vector av tm args"))
   (send* self :angle-vector-motion-plan av :ctype ctype :move-arm :rarm :total-time tm
               :start-offset-time (if start-offset-time start-offset-time start-time)
               :clear-velocities clear-velocities :use-torso use-torso args)))
  (:angle-vector-sequence
   (avs &optional tms &rest args) ;; (ctype controller-type) (start-time 0) &rest args
                                  ;; &key (use-torso t) (clear-velocities t) &allow-other-keys)
   "Send joind angle to robot with self-collision motion planning, this method returns immediately, so use :wait-interpolation to block until the motion stops.
- avs : sequence of joint angle vector [rad]
- tms : list of time to goal from previous angle-vector point in [msec]
- use-torso : set t to use torso
"
   (let ((ctype controller-type) (start-time 0) (start-offset-time nil) (use-torso t) (clear-velocities t))
   ;; as of 0.3.x, :angle-vector (robot-interface) :acceps tm ctype start-time as optional arguments, but in here we prefer old API
   (if (= (length args) 1) ;; args must be ctype
       (setq ctype (car args)
             args (cdr args)))
   (if (and (>= (length args) 2) (null (member (car args) '(:use-torso :start-time :clear-velocities))));; args must be ctype start-time
       (setq ctype (car args)
             start-time (cadr args)
             args (cddr args)))
   (if (member :use-torso args) (setq use-torso (cadr (member :use-torso args))))
   (if (member :start-time args) (setq use-torso (cadr (member :start-time args))))
   (if (member :start-offset-time args) (setq start-offset-time (cadr (member :start-offset-time args))))
   (if (member :clear-velocities args) (setq clear-velocities (cadr (member :clear-velocities args))))
   ;; for simulation mode
   (when (send self :simulation-modep)
     (if (member :use-torso args)
       (setq args (append (subseq args 0 (position :use-torso args))
                          (if (> (length args) (+ 2 (position :use-torso args)))
                            (subseq args (+ (position :use-torso args) 2))))))
     (return-from :angle-vector-sequence
                  (send* self :angle-vector-sequence-raw avs tms ctype start-time args)))
   (unless (and (listp tms) (every #'numberp tms))
     (ros::warn ":angle-vector-sequence tms is not a list of number, use :angle-vector-sequence av tms args"))
   (if tms
     (setq tms (apply #'+ tms))
     (setq tms 3000))
   (send* self :angle-vector-motion-plan avs :ctype ctype :move-arm :rarm :total-time tms
               :start-offset-time (if start-offset-time start-offset-time start-time)
               :clear-velocities clear-velocities :use-torso use-torso args)))
  (:default-controller ()
   (append
    (send self :arm-controller)
    (send self :torso-controller)
    (send self :head-controller)))
  (:arm-controller ()
   (list
    (list
     (cons :controller-action "/arm_controller/follow_joint_trajectory")
     (cons :controller-state "/arm_controller/follow_joint_trajectory/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "shoulder_pan_joint" "shoulder_lift_joint" "upperarm_roll_joint" "elbow_flex_joint" "forearm_roll_joint" "wrist_flex_joint" "wrist_roll_joint")))))
  (:torso-controller ()
   (list
    (list
     (cons :controller-action "/torso_controller/follow_joint_trajectory")
     (cons :controller-state "/torso_controller/follow_joint_trajectory/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "torso_lift_joint")))))
  (:head-controller ()
   (list
    (list
     (cons :controller-action "/head_controller/follow_joint_trajectory")
     (cons :controller-state "/head_controller/follow_joint_trajectory/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "head_pan_joint" "head_tilt_joint")))))
  (:start-grasp
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp :pos 0 args))
  (:stop-grasp
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp :pos 0.1 args))
  (:go-grasp
    (&key (pos 0) (effort 50) (wait t))
    (when (send self :simulation-modep)
      (send robot :l_gripper_finger_joint :joint-angle (/ (* pos 1000) 2)) ;; m -> mm
      (send robot :r_gripper_finger_joint :joint-angle (/ (* pos 1000) 2))
      (return-from :go-grasp t))
    (let (goal result)
      (setq goal (instance control_msgs::GripperCommandActionGoal :init))
      (send goal :goal :command :position pos)
      (send goal :goal :command :max_effort effort)
      (send gripper-action :send-goal goal)
      (when wait (send gripper-action :wait-for-result))
      (setq result (send gripper-action :get-result))
      result))
  (:fetch-gripper-state-callback
    (msg)
    (let ((msg (car (send msg :joints))))
      (dolist (slot '((position . 2) (velocity . 2) (effort . 1)))
	(send self :set-robot-state1 (intern (format nil "GRIPPER-~A" (string (car slot))) *keyword-package*) (* (cdr slot) (send msg (intern (string (car slot)) *keyword-package*)))))))
  (:gripper (key)
   "get information about gripper
Arguments:
 - key (:position :velocity :effort)
Example: (send self :gripper :position) => 0.00"
   (if (send self :simulation-modep)
       (let ((pos (* 0.001 2 (vmean (send robot :rarm :gripper :angle-vector)))))
         (case key
               (:position pos)
               (:velocity 0)
               (:effort (+ (* -110 pos) 1)))) ;; effot is -10 on open, 1.2 for close
     ;; for real robot
     (send-super :state (intern (format nil "GRIPPER-~A" (string key)) *keyword-package*))))
  ;;
  (:speak (text &rest args &key (topic-name "sound_play") &allow-other-keys)
    (send-super* :speak text :topic-name topic-name args))
  (:speak-en (text &rest args &key (topic-name "sound_play") &allow-other-keys)
    (send-super* :speak-en text :topic-name topic-name args))
  (:speak-jp (text &rest args &key (topic-name "robotsound_jp") &allow-other-keys)
    (send-super* :speak-jp text :topic-name topic-name args))
  (:set-workspace (x y z)
    (let ((bound (make-cube (+ x 50) (+ y 50) (+ z 50)))
          (inner-cube (make-cube x y z)))
      (setq bound (body- bound inner-cube))
      (send bound :translate (float-vector 0 0 (- (/ z 2) 10)))
      (send *co* :add-attached-object bound
            "base_link"
            :frame-id base-frame-id
            :object-id "workspace"
            :touch-links nil)))
  (:delete-workspace ()
    (send *co* :delete-attached-object-by-id "workspace")
    (send *co* :delete-object-by-id "workspace"))
  (:add-headbox-collision-object ()
    (let ()
      ;; fetch must be :reset-pose when we run this method
      (setq *fetch-headbox* (make-cube 100 201 120))
      (send *fetch-headbox* :move-coords (send robot :head_pan_link_lk :worldcoords)
                                         (send robot :base_link_lk :worldcoords))
      (send *fetch-headbox* :translate #f(33.75 0 150) (send robot :head_pan_link_lk :worldcoords))
      (send *co* :add-attached-object *fetch-headbox* "head_pan_link"
                 :frame_id "head_pan_link"
                 :object_id "fetchheadbox")))
  (:delete-headbox-collision-object ()
    (send *co* :delete-attached-object-by-id "fetchheadbox"))
  (:add-keepout-collision-object ()
    (let ((cube (make-cube 200 350 10))
          (keepout (make-cylinder 300 10)))
      (send cube :translate #f(-120 0 0))
      (send keepout :translate #f(0 0 -5))
      (setq keepout (body- keepout cube))
      (send keepout :translate #f(0 0 370))
      (send *co* :add-attached-object keepout
            "base_link"
            :frame-id base-frame-id
            :object-id "keepout"
            :touch-links (list "base_link" "torso_fixed_link"))))
  (:delete-keepout-collision-object ()
    (send *co* :delete-attached-object-by-id "keepout")
    (send *co* :delete-object-by-id "keepout"))
  (:add-ground-collision-object ()
    (let ((cube (make-cube 500 500 50))
          (ground (make-cylinder 1000 50)))
      (send cube :translate #f(-20 0 0))
      (send ground :translate #f(0 0 -25))
      (setq ground (body- ground cube))
      (send ground :translate #f(0 0 -25))
      (send *co* :add-attached-object ground
            "base_link"
            :frame-id base-frame-id
            :object-id "ground"
            :touch-links nil)))
  (:delete-ground-collision-object ()
    (send *co* :delete-attached-object-by-id "ground")
    (send *co* :delete-object-by-id "ground"))
  (:servo-off (&key (arm t) (gripper t) (head t))
    "
    Turn servo off for arm/gripper/head motors
    Args:
      - key
        - arm: If you do not want to servo off the arm, set nil (default: t)
        - gripper: If you do not want to servo off the grippper, set nil (default: t)
        - head: If you do not want to servo off the head, set nil (default: t)
    Examples:
      (send self :servo-off) ;; arm, gripper and  head servos off
      (send self :servo-off :arm t :gripper nil :head nil) ;; arm servo off
    "
    (let ((goal-servo-off (instance robot_controllers_msgs::QueryControllerStatesGoal :init))
          (goal-gripper-servo-off (instance control_msgs::GripperCommandGoal :init))
          (state nil) (update-list nil)
          (gravity-comp (list "arm_controller/gravity_compensation"))
          (arm-controller (list "arm_controller/follow_joint_trajectory"
                                "arm_with_torso_controller/follow_joint_trajectory"
                                "torso_controller/follow_joint_trajectory"))
          (head-controller (list "head_controller/follow_joint_trajectory"
                                 "head_controller/point_head")))
      (when (or arm head)
        (setq update-list (send goal-servo-off :updates))
        (when arm
          (ros::ros-info "arm servo off")
          ;; start gravity compensation
          (dolist (controller gravity-comp)
            (setq state (instance robot_controllers_msgs::ControllerState :init))
            (send state :name controller)
            (send state :state 1) ;; running
            (push state update-list))
          ;; stop arm controllers
          (dolist (controller arm-controller)
            (setq state (instance robot_controllers_msgs::ControllerState :init))
            (send state :name controller)
            (send state :state 0) ;; stopping
            (push state update-list)))
        (when head
          ;; stop head controllers
          (ros::ros-info "head servo off")
          (dolist (controller head-controller)
            (setq state (instance robot_controllers_msgs::ControllerState :init))
            (send state :name controller)
            (send state :state 0) ;; stopping
            (push state update-list)))
        (send goal-servo-off :updates update-list)
        (send fetch-controller-action :send-goal goal-servo-off))
      (when gripper
        ;; disable gripper torque
        (ros::ros-info "gripper servo off")
        (send (send goal-gripper-servo-off :command) :max_effort -1.0)
        (send gripper-action :send-goal goal-gripper-servo-off))))
  (:servo-on (&key (arm t) (gripper t) (head t) (time 200))
    "
    Turn servo on for arm/gripper/head motors
    Args:
      - key:
        - arm: If you do not want to servo on the arm, set nil (default: t)
        - gripper: If you do not want to servo on the grippper, set nil (default: t)
        - head: If you do not want to servo on the head, set nil (default: t)
        - time: [msec] duration to turn servo on,
                experimentally the minimal duration is about 60[msec] (default: 200)
    Examples:
      (send self :servo-on) ;; arm, gripper and  head servos on
      (send self :servo-on :arm nil :gripper nil :head t :time 80) ;; head servo on in 80[msec]
    "
    ;; the reason why :servo-off and :servo-on has different implementation is in the following issue
    ;; https://github.com/fetchrobotics/robot_controllers/issues/72
    (when arm
      (ros::ros-info "arm servo on")
      (send self :angle-vector (send self :state :potentio-vector) time :arm-controller)
      (send self :angle-vector (send self :state :potentio-vector) time :torso-controller))
    (when head
      (ros::ros-info "head servo on")
      (send self :angle-vector (send self :state :potentio-vector) time :head-controller))
    (when gripper
      (ros::ros-info "gripper servo on")
      (send self :go-grasp :pos (send self :gripper :position)))))

;; interface for simple base actions
(defmethod fetch-interface
  (:send-cmd-vel-raw (x d &key (topic-name "/base_controller/command"))
    (when (send self :simulation-modep)
      (return-from :send-cmd-vel-raw t))
    (unless (ros::get-topic-publisher topic-name)
      (ros::advertise topic-name geometry_msgs::Twist 1)
      (unix:sleep 1))
    (let ((msg (instance geometry_msgs::Twist :init)))
      (send msg :linear :x x)
      (send msg :angular :z d)
      (ros::publish topic-name msg)))
  (:go-velocity
    (x y d ;; [m/sec] [m/sec] [rad/sec]
       &optional (msec 1000) ;; msec is total animation time [msec]
       &key (stop t) (wait t))
    (unless (zerop y)
      (ros::ros-error
        "Setting velocity in y-axis is not valid for non-holonomic robot")
      (return-from :go-velocity nil))
    (unless wait
      (ros::ros-error ":go-velocity without wait is unsupported")
      (return-from :go-velocity nil))
    (setq msec (- msec (ros::get-param "/base_controller/timeout" 0.25)))
    (ros::rate 100)
    (let ((start-time (ros::time-now)))
      (while (and (ros::ok)
                  (< (* 1000.0 (send (ros::time- (ros::time-now) start-time) :to-sec)) msec))
        (send self :spin-once)
        (send self :send-cmd-vel-raw x d)
        (ros::sleep)))
    (when stop
      (send self :send-cmd-vel-raw 0 0))
    (ros::rate 10)
    t)
  (:go-pos-unsafe
    (x y d &rest args) ;; [m] [m] [degree]
    (send self :put :go-pos-unsafe-no-wait-goal (float-vector x y d))
    (send* self :go-pos-unsafe-wait args)
    t)
  (:go-pos-unsafe-no-wait
    (x y &optional (d 0)) ;; [m] [m] [degree]
    (ros::ros-warn ":go-pos-unsafe-no-wait is not supported for this robot.")
    (send self :put :go-pos-unsafe-no-wait-goal (float-vector x y d))
    t)
  (:go-pos-unsafe-wait
    (&key (translation-threshold 0.05) (rotation-threshold (deg2rad 5))
          (translation-gain 1.0) (rotation-gain 1.0)
          (min-translation-abs-vel 0.3) (min-rotation-abs-vel 0.8))
    (unless (send self :get :go-pos-unsafe-no-wait-goal)
      (ros::ros-error ":go-pos-unsafe-wait is called without goal")
      (return-from :go-pos-unsafe-wait nil))
    (ros::rate 100)
    (labels ((shortest-angle
              (d0 d1)
              (atan2 (sin (- d0 d1)) (cos (- d0 d1)))))
      (let* ((goal (send self :get :go-pos-unsafe-no-wait-goal))
             (x (aref goal 0))
             (y (aref goal 1))
             (d (aref goal 2))
             (forward-p (>= x 0))
             (d0 (if forward-p (atan2 y x) (atan2 (- y) (- x))))
             (d1 (distance (float-vector 0 0) (float-vector x y)))
             (d2 (shortest-angle (- (deg2rad d) d0) 0))
             org-cds cur-cds diffrot diffpos err)
        (send self :remprop :go-pos-unsafe-no-wait-goal)

        ;; yaw-x-yaw approach
        (ros::ros-info "go-pos-unsafe: (x y d) = (~A ~A ~A)" x y d)
        (ros::ros-info "go-pos-unsafe: (yaw, x, yaw) = (~A ~A ~A)"
                       d0 (* (if forward-p 1.0 -1.0) d1) d2)

        ;; yaw
        (send self :spin-once)
        (setq org-cds (send self :state :odom :pose))
        (ros::ros-debug "org-cds: ~A" org-cds)
        (while (ros::ok)
          (send self :spin-once)
          (setq cur-cds (send self :state :odom :pose))
          (setq diffrot (aref (send org-cds :difference-rotation cur-cds) 2))
          (setq err (shortest-angle d0 diffrot))
          (ros::ros-debug "diffrot: ~A, err: ~A" diffrot err)
          (when (< (abs err) rotation-threshold)
            (unless (send self :simulation-modep)
              (send self :send-cmd-vel-raw 0 0)) ;; stop
            (return))
          (when (send self :simulation-modep)
            (send self :robot :rotate (deg2rad (if (> err 0) 5 -5)) :z)
            (send self :draw-objects))
          (let ((d-vel (* rotation-gain err)))
            (send self :send-cmd-vel-raw 0
              (if (> (abs d-vel) min-rotation-abs-vel)
                d-vel (* (if (>= d-vel 0) 1.0 -1.0) min-rotation-abs-vel))))
          (ros::sleep))

        ;; x
        (setq org-cds (send self :state :odom :pose))
        (while (ros::ok)
          (send self :spin-once)
          (setq cur-cds (send self :state :odom :pose))
          (setq diffpos (* 0.001 (norm (send org-cds :difference-position cur-cds))))
          (setq err (- d1 diffpos))
          (ros::ros-debug "diffpos: ~A, err: ~A" diffpos err)
          (when (< (abs err) translation-threshold)
            (send self :send-cmd-vel-raw 0 0) ;; stop
            (return))
          (when (send self :simulation-modep)
            (send self :robot :newcoords
                  (send (send self :robot :copy-worldcoords)
                        :translate (float-vector
                                    (if forward-p 0.1 -0.1) 0 0)))
            (send self :draw-objects))
          (let ((x-vel (* translation-gain (if forward-p err (- err)))))
            (send self :send-cmd-vel-raw
              (if (> (abs x-vel) min-translation-abs-vel)
                x-vel (* (if (>= x-vel 0) 1.0 -1.0) min-translation-abs-vel)) 0))
          (ros::sleep))

        ;; yaw
        (setq org-cds (send self :state :odom :pose))
        (while (ros::ok)
          (send self :spin-once)
          (setq cur-cds (send self :state :odom :pose))
          (setq diffrot (aref (send org-cds :difference-rotation cur-cds) 2))
          (setq err (shortest-angle d2 diffrot))
          (ros::ros-debug "diffrot: ~A, err: ~A" diffrot err)
          (when (< (abs err) rotation-threshold)
            (send self :send-cmd-vel-raw 0 0) ;; stop
            (return))
          (when (send self :simulation-modep)
            (send self :robot :rotate (deg2rad (if (> err 0) 5 -5)) :z)
            (send self :draw-objects))
          (let ((d-vel (* rotation-gain err)))
            (send self :send-cmd-vel-raw 0
              (if (> (abs d-vel) min-rotation-abs-vel)
                d-vel (* (if (>= d-vel 0) 1.0 -1.0) min-rotation-abs-vel))))
          (ros::sleep))
        t)))
  (:move-to-wait (&rest args &key (correction nil) &allow-other-keys)
                 (send-super* :move-to-wait :correction correction args))
) ;; fetch-interface (simple base actions)

(defun fetch-init (&optional (create-viewer))
  (unless (boundp '*fetch*) (fetch) (send *fetch* :reset-pose))
  (unless (ros::ok) (ros::roseus "fetch_eus_interface"))
  (unless (boundp '*co*)
    (ros::ros-warn ";; collision-object-publisher wait for \"apply_planning_scene\" service for ~A sec~%" 5)
    (if (ros::wait-for-service "apply_planning_scene" 5)
      (setq *co* (instance collision-object-publisher :init))
      (ros::ros-warn ";; could not find \"apply_planning_scene\" skip creating *co*~%")))
  (unless (boundp '*ri*) (setq *ri* (instance fetch-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)

  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (when create-viewer (objects (list *fetch*)))
  )

(defclass fetch-moveit-environment
  :super moveit-environment)
(defmethod fetch-moveit-environment
  (:init (&key ((:robot rb) *fetch*) &rest args)
         (send-super* :init :robot rb :frame-id "base_link" args))
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "arm")
               (cons :target-link
                     (send self :search-link-from-name "wrist_roll_link"))
               (cons :joint-list (send robot :rarm :joint-list))
               )
         (list :rarm-torso
               (cons :group-name "arm_with_torso")
               (cons :target-link
                     (send self :search-link-from-name "wrist_roll_link"))
               (cons :joint-list (append
                                  (send robot :torso :joint-list)
                                  (send robot :rarm :joint-list)))
               )
#|
    <group name="arm">
        <joint name="shoulder_pan_joint" />
        <joint name="shoulder_lift_joint" />
        <joint name="upperarm_roll_joint" />
        <joint name="elbow_flex_joint" />
        <joint name="forearm_roll_joint" />
        <joint name="wrist_flex_joint" />
        <joint name="wrist_roll_joint" />
    </group>
    <group name="arm_with_torso">
        <joint name="torso_lift_joint" />
        <joint name="shoulder_pan_joint" />
        <joint name="shoulder_lift_joint" />
        <joint name="upperarm_roll_joint" />
        <joint name="elbow_flex_joint" />
        <joint name="forearm_roll_joint" />
        <joint name="wrist_flex_joint" />
        <joint name="wrist_roll_joint" />
    </group>
    <!--END EFFECTOR: Purpose: Represent information about an end effector.-->
    <end_effector name="gripper" parent_link="wrist_roll_link" group="gripper" />
|#
         ))
  )

#|
(setq *ri* (instance fetch-interface :init))
(setq *fetch* (fetch))
(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
|#


(provide :fetch-interface)
