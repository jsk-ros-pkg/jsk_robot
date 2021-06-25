(require "package://kinovaeus/kinova-utils.l")
(require "package://pr2eus/robot-interface.l")
(require "package://pr2eus_moveit/euslisp/robot-moveit.l")
(ros::load-ros-manifest "kortex_driver")

(if (not (find-package "kinova"))
    (make-package "kinova"))

(defclass kinova-interface
  :super robot-interface
  :slots (controller-handle robot-handle move-robot prefix)
  )

(defmethod kinova-interface
  (:init
   (&key type (prefix-arg "") &rest args)
   (let ()
     (setq prefix prefix-arg)
     (prog1
         (send-super* :init
                      :robot
                      ;; Create robot instance different from *kinova*
                      (case type
                            (:gen3_robotiq_2f_85
                             (instance gen3_robotiq_2f_85-robot :init))
                            (:gen3_robotiq_2f_140
                             (instance gen3_robotiq_2f_140-robot :init))
                            (:gen3_lite_gen3_lite_2f
                             (instance gen3_lite_gen3_lite_2f-robot :init))
                            (t
                             (warning-message 1 "unknown kinova robot type ~A~%" type)))
                      :joint-states-topic "/arm_gen3/joint_states"
                      :groupname "kinova_interface" args)
       (send self :add-controller :arm-controller)
       (setq gripper-action
             (instance ros::simple-action-client :init
                       (cond ((derivedp robot gen3_robotiq_2f_140-robot)
                              (format nil "/arm_gen3/~Arobotiq_2f_140_gripper_controller/gripper_cmd" prefix))
                             ((derivedp robot gen3_robotiq_2f_85-robot)
                              (format nil "/arm_gen3/~Arobotiq_2f_85_gripper_controller/gripper_cmd" prefix))
                             ((derivedp robot gen3_lite_gen3_lite_2f-robot)
                              (format nil "/arm_gen3/~Agen3_lite_2f_gripper_controller/gripper_cmd" prefix))
                             (t
                              (warning-message 1 "unknown kinova robot type ~A~%" robot)
                              ))
                       control_msgs::GripperCommandAction
                       :groupname groupname))
       (setq moveit-robot (instance (eval (send (class robot) :name)) :init))
       (send self :set-moveit-environment (instance kinova-moveit-environment :init :robot moveit-robot))
       (ros::advertise "/arm_gen3/in/cartesian_velocity" kortex_driver::TwistCommand)
       (ros::advertise "/arm_gen3/in/joint_velocity" kortex_driver::Base_JointSpeeds)
       (ros::advertise "/arm_gen3/in/clear_faults" std_msgs::Empty)
       (ros::advertise "/arm_gen3/in/stop" std_msgs::Empty)
       (ros::advertise "/arm_gen3/in/emergency_stop" std_msgs::Empty)
       self)))
  ;; default robot-interface methods
  (:default-controller () (send self :arm-controller))
  (:arm-controller ()
    (list
     (append
      (cond ((or (derivedp robot gen3_robotiq_2f_140-robot) (derivedp robot gen3_robotiq_2f_85-robot))
             (list
              (cons :controller-action
                    (format nil "arm_gen3/~Agen3_joint_trajectory_controller/follow_joint_trajectory" prefix))
              (cons :controller-state
		    (format nil "arm_gen3/~Agen3_joint_trajectory_controller/state" prefix)))) ;; @TODO: This is dummy topoic name. The kinova_driver did not publish JointTrajectoryControllerState
            ((derivedp robot gen3_lite_gen3_lite_2f-robot)
             (list
              (cons :controller-action
                    (format nil "arm_gen3/~Agen3_lite_joint_trajectory_controller/follow_joint_trajectory" prefix))
              (cons :controller-state
		    (format nil "arm_gen3/~Agen3_lite_joint_trajectory_controller/state" prefix))))) ;; @TODO: This is dummy topoic name. The kinova_driver did not publish JointTrajectoryControllerState
      (list (cons :action-type control_msgs::FollowJointTrajectoryAction)
            (cons :joint-names (send-all (send robot :joint-list) :name))))))
  (:go-grasp
   (&rest args)
   (cond ((derivedp robot gen3_robotiq_2f_140-robot)
          (send* self :go-grasp-robotiq-2f-140 args))
         ((derivedp robot gen3_robotiq_2f_85-robot)
          (send* self :go-grasp-robotiq-2f-85 args))
         ((derivedp robot gen3_lite_gen3_lite_2f-robot)
          (send* self :go-grasp-lite args))
         (t
          (warning-message 1 "unknown kinova robot type ~A~%" robot))))
  (:start-grasp
   (&rest args)
   (cond ((derivedp robot gen3_robotiq_2f_140-robot)
          (send* self :start-grasp-robotiq-2f-140 args))
         ((derivedp robot gen3_robotiq_2f_85-robot)
          (send* self :start-grasp-robotiq-2f-85 args))
         ((derivedp robot gen3_lite_gen3_lite_2f-robot)
          (send* self :start-grasp-lite args))
         (t
          (warning-message 1 "unknown kinova robot type ~A~%" robot))))
  (:stop-grasp
   (&rest args)
   (cond ((derivedp robot gen3_robotiq_2f_140-robot)
          (send* self :stop-grasp-robotiq-2f-140 args))
         ((derivedp robot gen3_robotiq_2f_85-robot)
          (send* self :stop-grasp-robotiq-2f-85 args))
         ((derivedp robot gen3_lite_gen3_lite_2f-robot)
          (send* self :stop-grasp-lite args))
         (t
          (warning-message 1 "unknown kinova robot type ~A~%" robot))))
  ;; grasp for robotiq 2f 85
  (:go-grasp-robotiq-2f-85 (&key (pos 0.0) (wait t))
    (when (send self :simulation-modep)
      ;; mimic joint
      ;; https://github.com/Kinovarobotics/ros_kortex/blob/kinetic-devel/kortex_description/grippers/robotiq_2f_85/urdf/robotiq_arg2f_85_model_macro.xacro
      (let ((pos-deg (rad2deg pos)))
        (send robot :finger_joint :joint-angle pos-deg)
        (send robot :left_inner_finger_joint :joint-angle (* -1 pos-deg))
        (send robot :left_inner_knuckle_joint :joint-angle pos-deg)
        (send robot :right_inner_finger_joint :joint-angle (* -1 pos-deg))
        (send robot :right_inner_knuckle_joint :joint-angle pos-deg)
        (send robot :right_outer_knuckle_joint :joint-angle pos-deg))
      (return-from :go-grasp-robotiq-2f-85 t))
    (let ((pos-max 0.8) (pos-min 0.0))
      (when (or (< pos pos-min) (> pos pos-max))
      (ros::ros-warn (format nil ":pos ~A is out of range." pos))
      (setq pos (max pos-min (min pos pos-max)))))
    (let (goal result)
      (setq goal (instance control_msgs::GripperCommandActionGoal :init))
      (send goal :goal :command :position pos)
      (send gripper-action :send-goal goal)
      (when wait (send gripper-action :wait-for-result))
      (setq result (send gripper-action :get-result))
      result))
  (:start-grasp-robotiq-2f-85
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp-robotiq-2f-85 :pos 0.8 args))
  (:stop-grasp-robotiq-2f-85
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp-robotiq-2f-85 :pos 0.0 args))
  ;; grasp for robotiq 2f 140
  (:go-grasp-robotiq-2f-140 (&key (pos 0.0) (wait t))
    (when (send self :simulation-modep)
      ;; mimic joint
      ;; https://github.com/Kinovarobotics/ros_kortex/blob/kinetic-devel/kortex_description/grippers/robotiq_2f_140/urdf/robotiq_arg2f_140_model_macro.xacro
      (let ((pos-deg (rad2deg pos)))
        (send self :finger_joint :joint-angle pos-deg)
        (send self :left_inner_finger_joint :joint-angle pos-deg)
        (send self :left_inner_knuckle_joint :joint-angle (* -1 pos-deg))
        (send self :right_inner_finger_joint :joint-angle pos-deg)
        (send self :right_inner_knuckle_joint :joint-angle (* -1 pos-deg))
        (send self :right_outer_knuckle_joint :joint-angle (* -1 pos-deg)))
      (return-from :go-grasp-robotiq-2f-140 t))
    (let ((pos-max 0.69) (pos-min 0.0))
      (when (or (< pos pos-min) (> pos pos-max))
      (ros::ros-warn (format nil ":pos ~A is out of range." pos))
      (setq pos (max pos-min (min pos pos-max)))))
    (let (goal result)
      (setq goal (instance control_msgs::GripperCommandActionGoal :init))
      (send goal :goal :command :position pos)
      (send gripper-action :send-goal goal)
      (when wait (send gripper-action :wait-for-result))
      (setq result (send gripper-action :get-result))
      result))
  (:start-grasp-robotiq-2f-140
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp-robotiq-2f-140 :pos 0.69 args))
  (:stop-grasp-robotiq-2f-140
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp-robotiq-2f-140 :pos 0.0 args))
  ;; grasp for gen3 lite
  (:go-grasp-lite (&key (pos 0.0) (wait t))
    (when (send self :simulation-modep)
      ;; mimic joint
      ;; https://github.com/Kinovarobotics/ros_kortex/blob/kinetic-devel/kortex_description/grippers/gen3_lite_2f/urdf/gen3_lite_2f_macro.xacro
      (send robot :right_finger_bottom_joint :joint-angle
            (rad2deg pos))
      (send robot :right_finger_tip_joint :joint-angle
            (rad2deg (+ (* -0.676 pos) 0.149)))
      (send robot :left_finger_bottom_joint :joint-angle
            (rad2deg (* -1 pos)))
      (send robot :left_finger_tip_joint :joint-angle
            (rad2deg (+ (* -0.676 pos) 0.149)))
      (return-from :go-grasp-lite t))
    (let ((pos-max 0.95) (pos-min -0.05))
      (when (or (< pos pos-min) (> pos pos-max))
      (ros::ros-warn (format nil ":pos ~A is out of range." pos))
      (setq pos (max pos-min (min pos pos-max)))))
    (let (goal result)
      (setq goal (instance control_msgs::GripperCommandActionGoal :init))
      (send goal :goal :command :position pos)
      (send gripper-action :send-goal goal)
      (when wait (send gripper-action :wait-for-result))
      (setq result (send gripper-action :get-result))
      result))
  (:start-grasp-lite
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp-lite :pos -0.05 args))
  (:stop-grasp-lite
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp-lite :pos 0.95 args))
  ;; :angle-vector-xxx and :check-continuous-joint-move-over-180
  ;; are mainly copied from package://fetcheus/fetch-interface.l
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
  (:angle-vector-raw (av &optional (tm 3000) (ctype controller-type) (start-time 0) &rest args)
   (let* ((prev-av (send self :state :potentio-vector :wait-until-update t))
          (diff-av (v- av prev-av)))
     (when (send self :check-continuous-joint-move-over-180 diff-av)
       (let* (avs (minjerk (instance minjerk-interpolator :init))
                  (scale-av (send self :sub-angle-vector av prev-av))
                  dist div)
         (setq dist (abs (geo::find-extream (coerce diff-av cons) #'abs #'>=)))
         (setq div (round (/ dist 120.0)))
         (send minjerk :reset
               :position-list (list prev-av (v+ prev-av scale-av))
               :time-list (list tm))
         (send minjerk :start-interpolation)
         (send minjerk :pass-time (/ tm div))
         (dotimes (i div)
           (setq avs (append avs (list (send minjerk :pass-time (/ tm div))))))
         (send* self :angle-vector-sequence-raw avs (make-list div :initial-element (/ tm div)) ctype start-time args)
         (return-from :angle-vector-raw (car (last avs)))))
     (send-super* :angle-vector av tm ctype start-time :minjerk-interpolation t args)))
  (:angle-vector-sequence-raw
   (avs &optional (tms (list 3000)) (ctype controller-type) (start-time 0.1) &rest args)
   (send-super* :angle-vector-sequence avs tms ctype start-time :minjerk-interpolation t args))
  (:angle-vector
   (av &optional (tm 3000) &rest args) ;; (ctype controller-type) (start-time 0) &rest args
                                       ;;  &key (clear-velocities t) &allow-other-keys)
   "Send joind angle to robot with self-collision motion planning, this method retuns immediately, so use :wait-interpolation to block until the motion stops.
- av : joint angle vector [rad]
- tm : time to goal in [msec]
"
   (let ((ctype controller-type) (start-time 0) (start-offset-time nil) (clear-velocities t))
   ;; as of 0.3.x, :angle-vector (robot-interface) :acceps tm ctype start-time as optional arguments, but in here we prefer old API
   (if (= (length args) 1) ;; args must be ctype
       (setq ctype (car args)
             args (cdr args)))
   (if (and (>= (length args) 2) (null (member (car args) '(:start-time :clear-velocities))));; args must be ctype start-time
       (setq ctype (car args)
             start-time (cadr args)
             args (cddr args)))
   (if (member :start-time args) (setq start-time (cadr (member :start-time args))))
   (if (member :start-offset-time args) (setq start-offset-time (cadr (member :start-offset-time args))))
   (if (member :clear-velocities args) (setq clear-velocities (cadr (member :clear-velocities args))))
   ;; for simulation mode
   (when (and (not (boundp '*co*)) (send self :simulation-modep))
     (return-from :angle-vector (send* self :angle-vector-raw av tm ctype start-time args)))
   ;;
   (when (not (numberp tm))
     (ros::warn ":angle-vector tm is not a number, use :angle-vector av tm args"))
   (send* self :angle-vector-motion-plan av :ctype ctype :move-arm :rarm :total-time tm
               :start-offset-time (if start-offset-time start-offset-time start-time)
               :clear-velocities clear-velocities
               :minjerk-interpolation t args)))
  (:angle-vector-sequence
   (avs &optional tms &rest args) ;; (ctype controller-type) (start-time 0) &rest args
                                  ;; &key (clear-velocities t) &allow-other-keys)
   "Send joind angle to robot with self-collision motion planning, this method returns immediately, so use :wait-interpolation to block until the motion stops.
- avs : sequence of joint angle vector [rad]
- tms : list of time to goal from previous angle-vector point in [msec]
"
   (let ((ctype controller-type) (start-time 0) (start-offset-time nil) (clear-velocities t))
   ;; as of 0.3.x, :angle-vector (robot-interface) :acceps tm ctype start-time as optional arguments, but in here we prefer old API
   (if (= (length args) 1) ;; args must be ctype
       (setq ctype (car args)
             args (cdr args)))
   (if (and (>= (length args) 2) (null (member (car args) '(:start-time :clear-velocities))));; args must be ctype start-time
       (setq ctype (car args)
             start-time (cadr args)
             args (cddr args)))
   (if (member :start-offset-time args) (setq start-offset-time (cadr (member :start-offset-time args))))
   (if (member :clear-velocities args) (setq clear-velocities (cadr (member :clear-velocities args))))
   ;; for simulation mode
   (when (send self :simulation-modep)
     (return-from :angle-vector-sequence
                  (send* self :angle-vector-sequence-raw avs tms ctype start-time args)))
   (unless (and (listp tms) (every #'numberp tms))
     (ros::warn ":angle-vector-sequence tms is not a list of number, use :angle-vector-sequence av tms args"))
   (if tms
     (setq tms (apply #'+ tms))
     (setq tms 3000))
   (send* self :angle-vector-motion-plan avs :ctype ctype :move-arm :rarm :total-time tms
               :start-offset-time (if start-offset-time start-offset-time start-time)
               :clear-velocities clear-velocities
               :minjerk-interpolation t args)))
  (:joint-velocity (joint-vel)
   "Send cartesian joint velocity command.
joint-vel : joint velocity [rad/s]. length must equal to the number of joint.
"
   (let ((joint-length (length (send robot :angle-vector)))
         (bj (instance kortex_driver::Base_JointSpeeds :init))
         js)
     (when (not (eq (length joint-vel) joint-length))
       (ros::ros-error (format nil "joint-vel must be the length of ~A" joint-length))
       (return-from :joint-velocity))
     (dotimes (i joint-length)
       (push (instance kortex_driver::JointSpeed :init
                       :joint_identifier i
                       :value (elt joint-vel i))
             js))
     (send bj :joint_speeds (reverse js))
     (ros::publish "/arm_gen3/in/joint_velocity" bj)))
  (:stop-joint-velocity ()
    (send self :joint-velocity
          (make-list (length (send robot :angle-vector)) :initial-element 0)))
  (:cartesian-velocity (linear-vel angular-vel)
   "Send cartesian velocity command to end effector
- linear  : linear velocity [m/s]. 3-dim float-vector.
- angular : angular velocity [rad/s]. 3-dim float-vector.
"
   (let ((tc (instance kortex_driver::TwistCommand :init)))
     (send tc :twist :linear_x (elt linear-vel 0))
     (send tc :twist :linear_y (elt linear-vel 1))
     (send tc :twist :linear_z (elt linear-vel 2))
     (send tc :twist :angular_x (elt angular-vel 0))
     (send tc :twist :angular_y (elt angular-vel 1))
     (send tc :twist :angular_z (elt angular-vel 2))
     (ros::publish "/arm_gen3/in/cartesian_velocity" tc)))
  (:stop-cartesian-velocity ()
    (send self :cartesian-velocity #f(0 0 0) #f(0 0 0)))
  (:clear-faults ()
    (ros::publish "/arm_gen3/in/clear_faults" (instance std_msgs::Empty :init)))
  (:stop ()
    (ros::publish "/arm_gen3/in/stop" (instance std_msgs::Empty :init)))
  (:emergency-stop ()
    (ros::publish "/arm_gen3/in/emergency_stop" (instance std_msgs::Empty :init)))
  )

(defclass kinova-moveit-environment
  :super moveit-environment)
(defmethod kinova-moveit-environment
  (:init (&key ((:robot rb) *gen3*) &rest args)
         (send-super* :init
                      :robot rb
                      :frame-id "base_link"
                      :scene-service "arm_gen3/get_planning_scene"
                      :planning-service "arm_gen3/plan_kinematic_path"
                      :execute-service "arm_gen3/execute_kinematic_path"
                      :query-planner-interface-service "arm_gen3/query_planner_interface"
                      :planning-scene-world "arm_gen3/planning_scene_world"
                      :state-validity-service "arm_gen3/check_state_validity"
                      args))
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "arm")
               (cons :target-link
                     (send self :search-link-from-name "end_effector_link"))
               (cons :joint-list (send robot :arm :joint-list)))))
  )


(defun kinova-init (&key (type nil) (prefix-arg "") &rest args &alow-other-keys)
  (unless (boundp '*kinova*) (apply #'kinova (if type (list :type type) nil)) (send *kinova* :reset-pose))
  (unless (ros::ok) (ros::roseus "kinova_eus_interface"))
  (unless (boundp '*co*)
    (ros::ros-warn ";; collision-object-publisher wait for \"arm_gen3/apply_planning_scene\" service for ~A sec~%" 5)
    (if (ros::wait-for-service "arm_gen3/apply_planning_scene" 5)
      (setq *co* (instance collision-object-publisher
                           :init
                           :service-name "arm_gen3/apply_planning_scene"
                           :scene-service "arm_gen3/get_planning_scene"))
      (ros::ros-warn ";; could not find \"arm_gen3/apply_planning_scene\" skip creating *co*~%")))
  (unless (boundp '*ri*) (setq *ri* (instance kinova-interface :init :type type :prefix-arg prefix-arg)))

  (ros::spin-once)
  (send *ri* :spin-once)

  ;;(when create-viewer (objects (list *kinova*)))
  )

;; Remove defmethod robot-interface when below is merged
;; https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/456
(defmethod robot-interface
  (:angle-vector
   (av &optional (tm nil) (ctype controller-type) (start-time 0) &key (scale 1) (min-time 1.0) (end-coords-interpolation nil) (end-coords-interpolation-steps 10) (minjerk-interpolation nil))
   "Send joint angle to robot, this method returns immediately, so use :wait-interpolation to block until the motion stops.
- av : joint angle vector [deg]
- tm : (time to goal in [msec])
   if designated tm is faster than fastest speed, use fastest speed
   if not specified, it will use 1/scale of the fastest speed .
   if :fast is specified use fastest speed calculated from max speed
- ctype : controller method name
- start-time : time to start moving
- scale : if tm is not specified, it will use 1/scale of the fastest speed
- min-time : minimum time for time to goal
- end-coords-interpolation : set t if you want to move robot in cartesian space interpolation
- end-coords-interpolation-steps : number of divisions when interpolating end-coords
- minjerk-interpolation : set t if you want to move robot with minjerk interpolation
"
   (if end-coords-interpolation
     (return-from :angle-vector (send self :angle-vector-sequence (list av) (list tm) ctype start-time :scale scale :min-time min-time :end-coords-interpolation t :end-coords-interpolation-steps end-coords-interpolation-steps)))
   (setq ctype (or ctype controller-type))  ;; use default controller-type if ctype is nil
   (unless (gethash ctype controller-table)
     (warn ";; controller-type: ~A not found" ctype)
     (return-from :angle-vector))
   ;;Check and decide tm
   (let ((fastest-tm (* 1000 (send self :angle-vector-duration
                                   (send self :state :potentio-vector) av scale min-time ctype))))
     (cond
      ;;Fastest time Mode
      ((equal tm :fast)
       (setq tm fastest-tm))
      ;;Normal Number designated Mode
      ((numberp tm)
       (if (< tm fastest-tm)
           (setq tm fastest-tm)))
      ;;Safe Mode (Speed will be 5 * fastest-tm)
      ((null tm)
       (setq tm (* 5 fastest-tm)))
      ;;Error Not Good Input
      (t
       (ros::ros-error ":angle-vector tm is invalid.  args: ~A" tm)
       (error ":angle-vector tm is invalid. args: ~A" tm)))
     )
   ;; for simulation mode
   (when (send self :simulation-modep)
     (if av (send self :angle-vector-simulation av tm ctype)))
   ;; minjerk interpolation, 0.001 [sec] inverval
   (let (r trajpoints)
     (if minjerk-interpolation
       (progn
         (setq r (pos-list-interpolation (list (send robot :angle-vector) av)
                                         (list (/ tm 1000.0))
                                         0.001))
         ;; TODO: Set vel and acc
         (setq trajpoints
               (mapcar #'(lambda (pos vel tim acc)
                           ;; (list pos vel tim acc)) ;; ideal
                           (list pos (make-list (length vel) :initial-element 0) tim (make-list (length acc) :initial-element 0))) ;; temporary
                       (cadr (memq :data r))           ;; positions
                       (cadr (memq :velocity r))       ;; velocities
                       (cadr (memq :time r))           ;; duration
                       (cadr (memq :acceleration r)))) ;; accelerations
         (nreverse trajpoints))
       (setq trajpoints
             (list
              (list av                                         ;; positions
                    (instantiate float-vector (length av))     ;; velocities
                    (/ tm 1000.0)                              ;; duration
                    (instantiate float-vector (length av)))))) ;; accelerations
     (send robot :angle-vector av)
     (let ((cacts (gethash ctype controller-table)))
       (mapcar
        #'(lambda (action param)
            (send self :send-ros-controller
                  action (cdr (assoc :joint-names param)) ;; action server and joint-names
                  start-time  ;; start time
                  trajpoints))
        cacts (send self ctype))))
   av)
  (:angle-vector-sequence
   (avs &optional (tms (list 3000)) (ctype controller-type) (start-time 0.1) &key (scale 1) (min-time 0.0) (end-coords-interpolation nil) (end-coords-interpolation-steps 10) (minjerk-interpolation nil))
   "Send sequence of joint angle to robot, this method returns immediately, so use :wait-interpolation to block until the motion stops.
- avs: sequence of joint angles(float-vector) [deg],  (list av0 av1 ... avn)
- tms: sequence of duration(float) from previous angle-vector to next goal [msec],  (list tm0 tm1 ... tmn)
   if tms is atom, then use (list (make-list (length avs) :initial-element tms))) for tms
   if designated each tmn is faster than fastest speed, use fastest speed
   if tmn is nil, then it will use 1/scale of the fastest speed .
   if :fast is specified, use fastest speed calculated from max speed
- ctype : controller method name
- start-time : time to start moving
- scale : if tms is not specified, it will use 1/scale of the fastest speed
- min-time : minimum time for time to goal
- end-coords-interpolation : set t if you want to move robot in cartesian space interpolation
- end-coords-interpolation-steps : number of divisions when interpolating end-coords
- minjerk-interpolation : set t if you want to move robot with minjerk interpolation
   "
   (setq ctype (or ctype controller-type))  ;; use default controller-type if ctype is nil
   (unless (gethash ctype controller-table)
     (warn ";; controller-type: ~A not found" ctype)
     (return-from :angle-vector-sequence))
   (send self :spin-once) ;; for :state :potentio-vector
   (let ((st 0) (traj-points nil)
         (av-prev (send self :state :potentio-vector)) av av-next
         tm tm-next fastest-tm
         (vel (instantiate float-vector (length (car avs)))))
     (if (atom tms) (setq tms (make-list (length avs) :initial-element tms)))
     (cond
      ((< (length tms) (length avs))
         (nconc tms (make-list (- (length avs) (length tms)) :initial-element (car (last tms)))))
      ((> (length tms) (length avs))
       (ros::ros-warn "length of tms should be the same or smaller than avs")
       (setq tms (subseq tms 0 (length avs)))))
     (when end-coords-interpolation ;; set end-coords interpolation
       (setq min-time (/ (float min-time) end-coords-interpolation-steps))
       (let ((av-orig (send robot :angle-vector)) ;; initial av, restore at end
             (c-orig  (send robot :copy-worldcoords)) ;; inital coords, restore at end
             (av-prev-orig av-prev) ;; prev-av
	     (diff-prev (instantiate float-vector (length av-prev))) diff ;; for over 360 deg turns
             (limbs '(:larm :rarm :lleg :rleg)) ;; do not move :head and :torso by IK
             target-limbs
             (minjerk (instance minjerk-interpolator :init))
             end-coords-prev end-coords-current ec-prev ec-current
             interpolated-avs interpolated-tms
             tm pass-tm i p (ret t)) ;; if nil failed to interpolate
           ;; set prev-av
         (send robot :angle-vector av-prev)
         (setq end-coords-prev (mapcar #'(lambda (limb) (send robot limb :end-coords :copy-worldcoords)) limbs))
         ;; choose moved end-coords
         (setq i 0)
         (dolist (av avs)
           (send robot :angle-vector av)
           (setq end-coords-current (mapcar #'(lambda (limb) (send robot limb :end-coords :copy-worldcoords)) limbs))
           (setq diff (v- (v- av (setq av (v+ av-prev (send self :sub-angle-vector av av-prev)))) diff-prev))
           (setq target-limbs nil)
           (setq tm (elt tms i))
           (if (equal tm :fast)
               (setq tm (* 1000 (send self :angle-vector-duration
                                      av-prev av scale min-time ctype))))
           (setq pass-tm (/ tm (float end-coords-interpolation-steps)))
           (dotimes (l (length limbs))
             (setq ec-prev (elt end-coords-prev l) ec-current (elt end-coords-current l))
             (when (and ec-prev ec-current
                        (or (> (norm (send ec-prev :difference-position ec-current)) 1)
                            (> (norm (send ec-prev :difference-rotation ec-current)) (deg2rad 1))))
               (push (elt limbs l) target-limbs)))
           (send minjerk :reset :position-list (list #f(0) #f(1)) :time-list (list tm))
           (send robot :angle-vector av-prev)
           (if target-limbs
               (progn
                 (send minjerk :start-interpolation)
                 (send minjerk :pass-time pass-tm)
                 (while (progn (send minjerk :pass-time pass-tm) (send minjerk :interpolatingp))
                   (setq p (elt (send minjerk :position) 0))
                   ;; set midpoint of av as initial pose of IK
                   (send robot :angle-vector (midpoint p av-prev av))
                   (dolist (limb target-limbs)
                     (setq ec-prev (elt end-coords-prev (position limb limbs))
                           ec-current (elt end-coords-current (position limb limbs)))
                     (setq ret (and ret
                                    (send robot limb :inverse-kinematics (midcoords p ec-prev ec-current)))))
                   (push (v++ diff-prev (scale p diff) (send robot :angle-vector)) interpolated-avs)
                   (push pass-tm interpolated-tms)
                   )
               (push (v++ diff-prev diff av) interpolated-avs)
               (push pass-tm interpolated-tms)
	       )
             (progn
               (push av interpolated-avs)
               (push tm interpolated-tms)))
           (setq end-coords-prev end-coords-current)
           (setq av-prev av)
	   (setq diff-prev diff)
           (incf i)) ;; dolist (av avs)
         ;; restore states
         (setq avs (nreverse interpolated-avs) tms (nreverse interpolated-tms))
         (send robot :move-to c-orig :world)
         (send robot :angle-vector av-orig)
         (setq av-prev av-prev-orig)
         (unless ret
           (warning-message 1 ":angle-vector-sequence failed to generate end-coords-interpolation motion~%")
           (return-from :angle-vector-sequence nil))
         ))
     (prog1 ;; angle-vector-sequence returns avs
         avs
       ;; minjerk interpolation, 0.001 [sec] inverval
       (if minjerk-interpolation
         (let (r)
           (setq r (pos-list-interpolation (append (list (send robot :angle-vector)) avs)
                                           (mapcar #'(lambda (x) (/ x 1000.0)) tms)
                                           0.001))
           ;; TODO: Set vel and acc
           (setq traj-points
                 (mapcar #'(lambda (pos vel tim acc)
                             ;; (list pos vel tim acc)) ;; ideal
                             (list pos (make-list (length vel) :initial-element 0) tim (make-list (length acc) :initial-element 0))) ;; temporary
                         (cadr (memq :data r))           ;; positions
                         (cadr (memq :velocity r))       ;; velocities
                         (cadr (memq :time r))           ;; duration
                         (cadr (memq :acceleration r)))) ;; accelerations
           (nreverse traj-points))
         (while avs
           (setq av (pop avs))
           (setq fastest-tm (* 1000 (send self :angle-vector-duration av-prev av scale min-time ctype)))
           (setq tm (pop tms))
           (cond
            ((equal tm :fast)
             (setq tm fastest-tm))
            ((null tm)
             (setq tm (* 5 fastest-tm)))
            ((numberp tm)
             (if (< tm fastest-tm)
               (setq tm fastest-tm)))
          (t (ros::ros-error ":angle-vector-sequence tm is invalid.  args: ~A" tm)
             (error ":angle-vector-sequence tm is invalid. args: ~A" tm)))
         (if (car tms)
             (progn
               (setq tm-next ( car tms))
               (setq fastest-tm-next (* 1000 (send self :angle-vector-duration av (car avs) scale min-time ctype)))
               (cond
                ((equal tm-next :fast)
                 (setq tm-next fastest-tm-next))
                ((null tm)
                 (setq tm (* 5 fastest-tm)))
                ((numberp tm-next)
                 (if (< tm-next fastest-tm-next)
                     (setq tm-next fastest-tm-next)))
                (t (ros::ros-error ":angle-vector-sequence tm is invalid.  args: ~A" tm)
                   (error ":angle-vector-sequence tm is invalid. args: ~A" tm)))
               )
           (setq tm-next 1)
           )
         (if (and (setq av-next (car avs)) (> tm 0) (> tm-next 0))
             (let ((v0 (send self :sub-angle-vector av av-prev))
                   (v1 (send self :sub-angle-vector av-next av)))
               (dotimes (i (length vel))
                 (setf (elt vel i)
                       (if (>= (* (elt v0 i) (elt v1 i)) 0)
                           (* 0.5 (+ (* (/ 1000.0 tm) (elt v0 i))
                                     (* (/ 1000.0 tm-next) (elt v1 i))))
                         0.0)))
               )
           (fill vel 0))
         ;; for simulation mode
         (if (send self :simulation-modep)
           (send self :angle-vector-simulation av tm ctype))
         ;;
         ;; update only joints with in current controller instead of (send robot :angle-vector av)
         (unless (send self :simulation-modep)
           (let* ((joint-names (flatten (mapcar #'(lambda (c) (cdr (assoc :joint-names c))) (send self ctype))))
                (joint-ids (mapcar #'(lambda (joint-name) (position joint-name (send robot :joint-list) :test #'(lambda (n j) (equal n (send j :name))))) joint-names)))
           (mapcar #'(lambda (name id)
                       (if (and (send robot :joint name) id (> (length av) id))
                           (send (send robot :joint name) :joint-angle (elt av id))
                         (warning-message 3 "[robot-interface.l] (angle-vector-sequence) could not find joint-name '~A' (~A) or joint-id ~A (av length ~A)~%" name (send robot :joint name) id (length av))))
                   joint-names joint-ids)))
;	 (when (send self :simulation-modep)
;	   (send self :publish-joint-state)
;	   (if viewer (send self :draw-objects)))

           (push (list av
                       (copy-seq vel)  ;; velocities
                       (/ (+ st tm) 1000.0) ;; tm + duration
                       (instantiate float-vector (length av))) ;; accelerations
                 traj-points)
           (setq av-prev av)
           (incf st tm)))
       ;;
       (let ((cacts (gethash ctype controller-table)))
         (unless cacts
           (warn ";; controller-type: ~A not found" ctype)
           (return-from :angle-vector-sequence))
         (mapcar
          #'(lambda (action param)
              (send self :send-ros-controller
                    action (cdr (assoc :joint-names param)) ;; action server and joint-names
                    start-time  ;; start time
                    traj-points))
          cacts (send self ctype)))
       )))
  (:send-ros-controller
   (action joint-names starttime trajpoints)
   (when (send self :simulation-modep)
     (return-from :send-ros-controller nil))
   (if (and warningp
	    (not (yes-or-no-p (format nil "~C[3~CmAre you sure to move the real robot? (~A) ~C[0m" #x1b 49 (send action :name) #x1b))))
       (return-from :send-ros-controller nil))
   (dolist (name joint-names)
     (unless (send robot :joint name)
       (warning-message 1 "[robot-interface.l] (send-ros-controller) could not find joint-name '~A' (~A)~%" name (send robot :joint name))
       (return-from :send-ros-controller nil)))
   (let* ((goal (send action :make-goal-instance))
	  (goal-points nil)
	  (st (if (numberp starttime)
                  (ros::time+ (ros::time-now) (ros::time starttime))
                starttime))
	  (joints (mapcar #'(lambda (x)
			      (send robot (intern (string-upcase x) *keyword-package*)))
			  joint-names)))
     (send goal :header :seq 1)
     (send goal :header :stamp st)

     (cond
      ((equal (class goal) control_msgs::SingleJointPositionActionGoal)
       (let* ((joint (car joints))
	      (id (position joint (send robot :joint-list)))
	      (pos (deg2rad (elt (elt (car trajpoints) 0) id)))
	      )
	 (send goal :goal :position pos)
	 (send goal :goal :max_velocity 5)
	 (send self :spin-once)
	 )
       )
      (t
       (send goal :goal :trajectory :joint_names joint-names)
       (send goal :goal :trajectory :header :stamp st)
       (dolist (trajpt trajpoints)
	 (let* ((all-positions (elt trajpt 0))
		(all-velocities (elt trajpt 1))
		(duration (elt trajpt 2))
		(all-accelerations (elt trajpt 3))
		(positions (instantiate float-vector (length joint-names)))
		(velocities (instantiate float-vector (length joint-names)))
		(accelerations (instantiate float-vector (length joint-names))))
	   (dotimes (i (length joints))
	     (let* ((joint (elt joints i))
		    (id (position joint (send robot :joint-list)))
		    p v a)
	       (setq p (elt all-positions id)
		     v (elt all-velocities id)
		     a (elt all-accelerations id))
	       (cond
		((derivedp joint rotational-joint)
		 (setq p (deg2rad p))
		 (setq v (deg2rad v))
		 (setq a (deg2rad a)))
		(t
		 (setq p (* 0.001 p))
		 (setq v (* 0.001 v))
		 (setq a (* 0.001 a))))
	       (setf (elt positions i) p)
	       (setf (elt velocities i) v)
	       (setf (elt accelerations i) a)))
	   (push (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions positions
			   :velocities velocities
			   :accelerations accelerations
			   :time_from_start (ros::time duration))
		 goal-points)
	   ))
       (send self :spin-once)
       (send goal :goal :trajectory :points goal-points)
       ))
     (send action :send-goal goal)
     ))
  (:send-trajectory (joint-trajectory-msg
                     &key ((:controller-type ct) controller-type) (starttime 1) (minjerk-interpolation nil) &allow-other-keys)
   (unless (gethash ct controller-table)
     (warn ";; controller-type: ~A not found" ct)
     (return-from :send-trajectory))
   (mapcar
    #'(lambda (action param)
        (send self :send-trajectory-each
              action (cdr (assoc :joint-names param)) ;; action server and joint-names
              joint-trajectory-msg
              starttime
              minjerk-interpolation))
    (gethash ct controller-table) (send self ct)))
  (:send-trajectory-each
   (action joint-names traj &optional (starttime 0.2) (minjerk-interpolation nil))
   (let* ((jnames (send traj :joint_names))
          (ilst (mapcar #'(lambda (jn) (position jn jnames :test #'string=)) joint-names))
          points-lst)
     (when (some #'identity ilst)
       (setq ilst (mapcar #'(lambda (jn)
                              (let ((p (position jn jnames :test #'string=)))
                                (unless p
                                  (setq p (send robot (intern (string-upcase jn) *keyword-package*))))
                                p))
                          joint-names))
       (if minjerk-interpolation
         ;; minjerk interpolation, 0.001 [sec] inverval
         (let ((points (send traj :points)) (tm-prev 0.0) avs tms r)
           (setq avs (mapcar #'(lambda (x) (send x :positions)) points))
           (dolist (tm (mapcar #'(lambda (x) (send (send x :time_from_start) :to-sec)) points))
             (setq tms (append tms (list (- tm tm-prev))))
             (setq tm-prev tm))
           (setq r (pos-list-interpolation avs (cdr tms) 0.001))
           (setq points-lst
                 ;; TODO: Set vel and acc
                 (mapcar #'(lambda (pos vel acc tim)
                             (instance trajectory_msgs::JointTrajectoryPoint :init
                                       :positions pos
                                       ;; :velocities vel ;; ideal
                                       :velocities (make-list (length vel) :initial-element 0) ;; temporary
                                       ;; :accelerations acc ;; ideal
                                       :accelerations (make-list (length acc) :initial-element 0) ;; temporary
                                       :time_from_start (ros::time tim)))
                         (cadr (memq :data r))
                         (cadr (memq :velocity r))
                         (cadr (memq :acceleration r))
                         (cadr (memq :time r))))
           (nreverse points-lst)
           )
         (dolist (p (send traj :points))
           (let ((poss (send p :positions))
                 (vels (send p :velocities))
                 (effs (send p :accelerations))
                 plst vlst elst)
             (dolist (i ilst)
               (cond
                ((numberp i)
                 (push (elt poss i) plst)
                 (if (and vels (> (length vels) i)) (push (elt vels i) vlst))
                 (if (and effs (> (length effs) i)) (push (elt effs i) elst)))
                (t
                 (push (send i :ros-joint-angle) plst)
                 (if vels (push 0 vlst))
                 (if effs (push 0 elst))
                 (ros::ros-warn ";; trajectory contains lacking joint names")
                 )))
             (push
              (instance trajectory_msgs::JointTrajectoryPoint :init
                        :positions (coerce (nreverse plst) float-vector)
                        :velocities (if vels (coerce (nreverse vlst) float-vector))
                        :accelerations (if effs (coerce (nreverse elst) float-vector))
                        :time_from_start (send p :time_from_start)) points-lst)
             ))
         )
       (let ((goal (send action :make-goal-instance))
             (st (ros::time+ (ros::time-now) (ros::time starttime))))
         (send goal :header :stamp (ros::time-now))
         (send goal :header :seq 1)
         (send goal :goal :trajectory :header :stamp st)
         (send goal :goal :trajectory :header :seq 1)
         (send goal :goal :trajectory :joint_names joint-names)
         (send goal :goal :trajectory :points (nreverse points-lst))
         (send self :spin-once)
         (send action :send-goal goal))
       (apply-trajectory_point joint-names (car (last points-lst)) robot)
       ) ;;; /when ilst
     ))
  )

;; Remove below when below is merged
;; https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/460
(defun controller-result-error-to-string (i)
  (elt '(SUCCESSFUL INVALID_GOAL INVALID_JOINTS OLD_HEADER_TIMESTAMP PATH_TOLERANCE_VIOLATED GOAL_TOLERANCE_VIOLATED) (* -1 i)))

(defmethod controller-action-client
  (:action-result-cb
   (msg)
   ;; The following error messages will not appear for the first (send *ri* :angle-vector)
   ;; To display it, use :wait-interpolation or (ros::spin-once groupname)
   (let ((error-code (send (send msg :result) :error_code)))
     (when (< error-code 0)
       ;; Remove the following two lines when below is merged
       ;; https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/460
       (ros::ros-error "~A~%" (send (send msg :result) :error_string))
       (ros::ros-error "error_code: ~A~%" (cons error-code (controller-result-error-to-string error-code)))
       (ros::ros-error "For detail, see https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_kinova_robot/README.md#trouble-shooting")))
   (send-super :action-result-cb msg)))

;; Remove below when below is merged
;; https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/456
(defun pos-list-interpolation
  (pos-list ;; (list pos_1 pos_2 ... pos_N), pos_i is float-vector
   time-list ;; (list dtime_1 dtime_2 ... dtime_{N-1}), dtime_i is time[s] between time at pos_{i+1} - pos_i
   dt ;; dt [s]
   &key (interpolator-class minjerk-interpolator)
        ((:interpolator ip) (instance interpolator-class :init))
        (initial-time 0.0) (neglect-first) (vel-vector-list) (acc-vector-list))
  (let* ((data-list) (tm-list) (vel-data-list) (acc-data-list))
    (assert (= (length pos-list) (1+ (length time-list)))
            (format nil "check length of pos-list(~A) and tm-list(~A)"
                    (length pos-list) (length time-list)))
    (setq vel-vector-list
          (reverse
           (do ((i 0 (1+ i)) (vel-list))
               ((> i (length time-list)) vel-list)
               (if (or (= i 0) (= i (length time-list)))
                   (push (instantiate float-vector (length (car pos-list))) vel-list)
                 (let* ((v0 (scale (/ 1.0  (elt time-list (1- i)))
                                   (v- (elt pos-list i) (elt pos-list (1- i)))))
                        (v1 (scale (/ 1.0 (elt time-list i))
                                   (v- (elt pos-list (1+ i)) (elt pos-list i))))
                        (v (scale 0.5 (v+ v0 v1))))
                   (dotimes (i (length v)) (if (< (* (elt v0 i) (elt v1 i)) 0) (setf (elt v i) 0)))
                   (push v vel-list))))))
    (setq acc-vector-list
          (reverse
           (do ((i 0 (1+ i)) (acc-list))
               ((> i (length time-list)) acc-list)
               (if (or (= i 0) (= i (length time-list)))
                   (push (instantiate float-vector (length (car vel-vector-list))) acc-list)
                 (let* ((v0 (scale (/ 1.0  (elt time-list (1- i)))
                                   (v- (elt vel-vector-list i) (elt vel-vector-list (1- i)))))
                        (v1 (scale (/ 1.0 (elt time-list i))
                                   (v- (elt vel-vector-list (1+ i)) (elt vel-vector-list i))))
                        (v (scale 0.5 (v+ v0 v1))))
                   (dotimes (i (length v)) (if (< (* (elt v0 i) (elt v1 i)) 0) (setf (elt v i) 0)))
                   (push v acc-list))))))
    ;; (format t "=INPUT~%")
    ;; (format t "time ~A~%" time-list)
    ;; (format t " pos ~A~%" pos-list)
    ;; (format t " vel ~A~%" vel-vector-list)
    ;; (format t " acc ~A~%" acc-vector-list)
    (send* ip :reset
           :position-list pos-list
           :time-list (let (r) (dolist (n time-list) (push (+ n (if r (car r) 0)) r)) (nreverse r)) ;; list of time[sec] from start for each control point
           (append
            (if vel-vector-list (list :velocity-list vel-vector-list))
            (if acc-vector-list (list :acceleration-list acc-vector-list))))
    (send ip :start-interpolation)
    (while (send ip :interpolatingp)
      (push (if (send ip :interpolatingp)
                (+ initial-time (send ip :time))
              (+ dt (car tm-list))) tm-list)
      (send ip :pass-time dt)
      (push (send ip :position) data-list)
      (if (find-method ip :velocity) (push (send ip :velocity) vel-data-list))
      (if (find-method ip :acceleration) (push (send ip :acceleration) acc-data-list))
      )
    ;; (format t "=OUTPUT~%")
    (if (and vel-data-list acc-data-list)
        (mapcar #'(lambda (tm pos vel acc)
                    ;; (format t "~7,5f ~7,3f ~13,1f ~13,1f~%"
                    ;;         tm (elt pos 0) (elt vel 0) (elt acc 0))
                    )
                (reverse tm-list) (reverse data-list) (reverse vel-data-list) (reverse acc-data-list)))
    (append
     (list :data (if neglect-first (cdr (reverse data-list)) (reverse data-list))
           :time (if neglect-first (cdr (reverse tm-list)) (reverse tm-list)))
     (if (find-method ip :velocity)
         (list :velocity (if neglect-first (cdr (reverse vel-data-list)) (reverse vel-data-list))))
     (if (find-method ip :acceleration)
         (list :acceleration (if neglect-first (cdr (reverse acc-data-list)) (reverse acc-data-list))))
     )))

;; This method can be removed if below is merged
;; https://github.com/euslisp/jskeus/pull/596
(defmethod interpolator
  (:pass-time
   (dt)
   "process interpolation for dt[sec]"
   (when interpolatingp
     (setq position (send self :interpolation))
     (incf time dt)
     (setq segment-time (- time (if (= segment 0) 0 (nth (1- segment) time-list))))
     (when (eps> time (nth segment time-list) (* 0.1 dt))
       ;; if time-segment is not aligned, need to fix the data (see https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/457)
       (while (and (< segment segment-num) (eps> time (nth segment time-list) (* 0.1 dt)))
         (setq segment-time (- time (nth segment time-list)))
         (incf segment)))
     (when (>= segment segment-num)
       ;; adjust time and segment-time to exact position
       (setq segment (1- segment-num))
       (setq time (car (last time-list)))
       (setq segment-time (- time (if (= segment 0) 0 (nth (1- segment) time-list))))
       ;; re-calculate :interpolation
       (setq position (send self :interpolation))
       (send self :reset))
     position))
  )

;; This method can be removed if below is merged
;; https://github.com/euslisp/jskeus/pull/596
(defmethod minjerk-interpolator
  (:interpolation
   ()
   "Minjerk interpolator, a.k.a Hoff & Arbib
    Example code is:
(setq l (instance minjerk-interpolator :init))
(send l :reset :position-list (list #f(1 2 3) #f(3 4 5) #f(1 2 3)) :time-list (list 0.1 0.18))
(send l :start-interpolation)
(while (send l :interpolatingp) (send l :pass-time 0.02) (print (send l :position)))
"
   (let* ((xi (nth segment position-list))
          (xf (nth (1+ segment) position-list))
          (vi (nth segment velocity-list))
          (vf (nth (1+ segment) velocity-list))
          (ai (nth segment acceleration-list))
          (af (nth (1+ segment) acceleration-list))
          ;;
          (t1+t2 (- (nth segment time-list) (if (> segment 0) (nth (1- segment) time-list) 0))) ;; total time of segment
          ;; A=(gx-(x+v*t+(a/2.0)*t*t))/(t*t*t)
          ;; B=(gv-(v+a*t))/(t*t)
          ;; C=(ga-a)/t
          (A  (scale (/ 1.0 (* t1+t2 t1+t2 t1+t2)) (v- xf (reduce #'v+ (list xi (scale t1+t2 vi) (scale (* t1+t2 t1+t2) (scale 0.5 ai)))))))
          (B  (scale (/ 1.0 (* t1+t2 t1+t2))      (v- vf (v+ vi (scale t1+t2 ai)))))
          (C  (scale (/ 1.0 (* t1+t2))           (v- af ai)))
          ;; a0=x
          ;; a1=v
          ;; a2=a/2.0
          ;; a3=10*A-4*B+0.5*C
          ;;; a4=(-15*A+7*B-C)/t
          ;; a5=(6*A-3*B+0.5*C)/(t*t)
          (a0 xi)
          (a1 vi)
          (a2 (scale 0.5 ai))
          (a3 (v+ (v- (scale 10 A) (scale 4 B)) (scale 0.5 C)))
          (a4 (scale (/ 1.0 t1+t2) (v- (v+ (scale -15 A) (scale 7 B)) C)))
          (a5 (scale (/ 1.0 t1+t2 t1+t2) (v+ (v+ (scale 6 A) (scale -3 B)) (scale 0.5 C))))
          )
     ;; x=a0+a1*t+a2*t*t+a3*t*t*t+a4*t*t*t*t+a5*t*t*t*t*t
     ;; v=a1+2*a2*t+3*a3*t*t+4*a4*t*t*t+5*a5*t*t*t*t
     ;; a=2*a2+6*a3*t+12*a4*t*t+20*a5*t*t*t
     (setq position
           (reduce #'v+ (list a0
                              (scale (expt segment-time 1) a1) (scale (expt segment-time 2) a2)
                              (scale (expt segment-time 3) a3) (scale (expt segment-time 4) a4)
                              (scale (expt segment-time 5) a5)))
           velocity
           (reduce #'v+ (list a1
                              (scale (* 2 (expt segment-time 1)) a2) (scale (* 3 (expt segment-time 2)) a3)
                              (scale (* 4 (expt segment-time 3)) a4) (scale (* 5 (expt segment-time 4)) a5)))
           acceleration
           (reduce #'v+ (list (scale 2 a2)
                              (scale (* 6 (expt segment-time 1)) a3) (scale (* 12 (expt segment-time 2)) a4)
                              (scale (* 20 (expt segment-time 3)) a5))))
     position))
  ;
  )
