(require "package://kinovaeus/kinova-utils.l")
(require "package://pr2eus/robot-interface.l")

(if (not (find-package "kinova"))
    (make-package "kinova"))

(defclass kinova-interface
  :super robot-interface
  :slots (controller-handle robot-handle move-robot)
  )

(defmethod kinova-interface
  (:init
   (&key type &rest args)
   (let ()
     (prog1
         (send-super* :init :robot (apply #'kinova (if type (list :type type) nil)) :joint-states-topic "/arm_gen3/joint_states" :groupname "kinova_interface" args)
       (send self :add-controller :arm-controller)
       (setq gripper-action
             (instance ros::simple-action-client :init
                       (cond ((derivedp robot gen3_robotiq_2f_140-robot)
                              "/arm_gen3/robotiq_2f_140_gripper_controller/gripper_cmd")
                             ((derivedp robot gen3_robotiq_2f_85-robot)
                              "/arm_gen3/robotiq_2f_85_gripper_controller/gripper_cmd")
                             ((derivedp robot gen3_robotiq_2f_140-robot)
                              "/arm_gen3/gen3_lite_2f_gripper_gripper_controller/gripper_cmd")
                             (t
                              (warning-message 1 "unknown kinova robot type ~A~%" robot)
                              ))
                       control_msgs::GripperCommandAction
                       :groupname groupname))
       (setq moveit-robot (instance (eval (send (class robot) :name)) :init))
       (send self :set-moveit-environment (instance gen3-moveit-environment :init :robot moveit-robot))
       self)))
  ;; default robot-interface methods
  (:default-controller () (send self :arm-controller))
  (:arm-controller ()
   (list
    (list
     (cons :controller-action "arm_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory")
     (cons :controller-state "arm_gen3/gen3_joint_trajectory_controller/state") ;; @TODO: This is dummy topoic name. The kinova_driver did not publish JointTrajectoryControllerState
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (send-all (send robot :joint-list) :name)))))
  (:go-grasp
   (&rest args)
   (cond ((derivedp robot gen3_robotiq_2f_140-robot)
          (send* self :go-grasp-robotiq args))
         ((derivedp robot gen3_robotiq_2f_85-robot)
          (send* self :go-grasp-robotiq args))
         ((derivedp robot gen3_robotiq_2f_140-robot)
          (send* self :go-grasp-gen3-lite args))
         (t
          (warning-message 1 "unknown kinova robot type ~A~%" robot))))
  (:start-grasp
   (&rest args)
   (cond ((derivedp robot gen3_robotiq_2f_140-robot)
          (send* self :start-grasp-robotiq args))
         ((derivedp robot gen3_robotiq_2f_85-robot)
          (send* self :start-grasp-robotiq args))
         ((derivedp robot gen3_robotiq_2f_140-robot)
          (send* self :start-grasp-gen3-lite args))
         (t
          (warning-message 1 "unknown kinova robot type ~A~%" robot))))
  (:stop-grasp
   (&rest args)
   (cond ((derivedp robot gen3_robotiq_2f_140-robot)
          (send* self :stop-grasp-robotiq args))
         ((derivedp robot gen3_robotiq_2f_85-robot)
          (send* self :stop-grasp-robotiq args))
         ((derivedp robot gen3_robotiq_2f_140-robot)
          (send* self :stop-grasp-gen3-lite args))
         (t
          (warning-message 1 "unknown kinova robot type ~A~%" robot))))
  ;; grasp for robotiq
  (:go-grasp-robotiq (&key (pos 0.0) (wait t))
    (when (send self :simulation-modep)
      (let ((pos-deg (rad2deg pos)))
        (send robot :finger_joint :joint-angle pos-deg)
        (send robot :left_inner_finger_joint :joint-angle (* -1 pos-deg))
        (send robot :left_inner_knuckle_joint :joint-angle pos-deg)
        (send robot :right_inner_finger_joint :joint-angle (* -1 pos-deg))
        (send robot :right_inner_knuckle_joint :joint-angle pos-deg)
        (send robot :right_outer_knuckle_joint :joint-angle pos-deg))
      (return-from :go-grasp-robotiq t))
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
  (:start-grasp-robotiq
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp-robotiq :pos 0.8 args))
  (:stop-grasp-robotiq
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp-robotiq :pos 0.0 args))
  ;; grasp for gen3 lite
  (:go-grasp-lite (&key (pos 0.0) (wait t))
    (when (send self :simulation-modep)
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
  (:angle-vector-raw (av &optional (tm 3000) &rest args)
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
         (send* self :angle-vector-sequence-raw avs (make-list div :initial-element (/ tm div)) args)
         (return-from :angle-vector-raw (car (last avs)))))
     (send-super* :angle-vector av tm args)))
  (:angle-vector-sequence-raw (&rest args) (send-super* :angle-vector-sequence args))
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
               :clear-velocities clear-velocities args)))
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
               :clear-velocities clear-velocities args)))
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

(defun kinova-init (&key (type nil) &rest args &alow-other-keys)
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
  (unless (boundp '*ri*) (setq *ri* (instance kinova-interface :init :type type)))

  (ros::spin-once)
  (send *ri* :spin-once)

  ;;(when create-viewer (objects (list *kinova*)))
  )
