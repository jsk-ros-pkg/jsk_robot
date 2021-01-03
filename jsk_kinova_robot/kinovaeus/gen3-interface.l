(require "package://kinovaeus/gen3-utils.l")
(require "package://pr2eus/robot-interface.l")
(require "package://pr2eus_moveit/euslisp/robot-moveit.l")

(defclass gen3-interface
  :super robot-interface
  :slots (gripper-action moveit-robot)
  )

(defmethod gen3-interface
  (:init
   (&rest args)
   (send-super* :init :robot gen3_robotiq_2f_85-robot :joint-states-topic "/my_gen3/joint_states" :groupname "gen3_interface" args)
   (send self :add-controller :arm-controller)
   (setq gripper-action
         (instance ros::simple-action-client :init
                   "/my_gen3/robotiq_2f_85_gripper_controller/gripper_cmd"
                   control_msgs::GripperCommandAction
                   :groupname groupname))
   (setq moveit-robot (instance gen3_robotiq_2f_85-robot :init))
   (send self :set-moveit-environment (instance gen3-moveit-environment :init :robot moveit-robot))
   )
  (:default-controller () (send self :arm-controller))
  (:arm-controller ()
   (list
    (list
     (cons :controller-action "my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory")
     (cons :controller-state "my_gen3/gen3_joint_trajectory_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "joint_1" "joint_2" "joint_3" "joint_4" "joint_5" "joint_6" "joint_7")))))
  (:go-grasp (&key (pos 0.0) (wait t))
    (when (send self :simulation-modep)
      (let ((pos-deg (rad2deg pos)))
        (send robot :finger_joint :joint-angle pos-deg)
        (send robot :left_inner_finger_joint :joint-angle (* -1 pos-deg))
        (send robot :left_inner_knuckle_joint :joint-angle pos-deg)
        (send robot :right_inner_finger_joint :joint-angle (* -1 pos-deg))
        (send robot :right_inner_knuckle_joint :joint-angle pos-deg)
        (send robot :right_outer_knuckle_joint :joint-angle pos-deg))
      (return-from :go-grasp t))
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
  (:start-grasp
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp :pos 0.8 args))
  (:stop-grasp
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp :pos 0.0 args))
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

(defun gen3-init (&optional (create-viewer))
  (unless (boundp '*gen3*) (gen3) (send *gen3* :reset-pose))
  (unless (ros::ok) (ros::roseus "gen3_eus_interface"))
  (unless (boundp '*co*)
    (ros::ros-warn ";; collision-object-publisher wait for \"my_gen3/apply_planning_scene\" service for ~A sec~%" 5)
    (if (ros::wait-for-service "my_gen3/apply_planning_scene" 5)
      (setq *co* (instance collision-object-publisher
                           :init
                           :service-name "my_gen3/apply_planning_scene"
                           :scene-service "my_gen3/get_planning_scene"))
      (ros::ros-warn ";; could not find \"my_gen3/apply_planning_scene\" skip creating *co*~%")))
  (unless (boundp '*ri*) (setq *ri* (instance gen3-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)

  (when create-viewer (objects (list *gen3*)))
  )

(defclass gen3-moveit-environment
  :super moveit-environment)
(defmethod gen3-moveit-environment
  (:init (&key ((:robot rb) *gen3*) &rest args)
         (send-super* :init
                      :robot rb
                      :frame-id "base_link"
                      :scene-service "my_gen3/get_planning_scene"
                      :planning-service "my_gen3/plan_kinematic_path"
                      :execute-service "my_gen3/execute_kinematic_path"
                      :query-planner-interface-service "my_gen3/query_planner_interface"
                      :planning-scene-world "my_gen3/planning_scene_world"
                      :state-validity-service "my_gen3/check_state_validity"
                      args))
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "arm")
               (cons :target-link
                     (send self :search-link-from-name "end_effector_link"))
               (cons :joint-list (send robot :arm :joint-list)))))
  )
