(require "package://pr2eus_moveit/euslisp/robot-moveit.l")
(require :baxter "package://baxtereus//baxter-util.l")
(load "package://pr2eus/speak.l")
(ros::load-ros-manifest "control_msgs")
(ros::load-ros-manifest "baxter_core_msgs")

(defparameter *suction* 65537) ;;vacuum
(defparameter *electoric* 65538)
(defvar *wait-for-suction* 5000000)

(defclass baxter-interface
  :super robot-interface
  :slots (gripper-sequence-id right-gripper-action left-gripper-action
			      right-gripper-type left-gripper-type moveit-robot))
(defmethod baxter-interface
  (:init (&rest args &key ((:moveit-environment mvit-env) (instance baxter-moveit-environment))
                ((:moveit-robot mvit-rb) (instance baxter-robot :init))
                ((:robot robot) baxter-robot)
                &allow-other-keys)
   (prog1 (send-super* :init :robot robot :joint-states-topic "/robot/joint_states" :groupname "baxter_interface" args)
     (send self :add-controller :larm-controller)
     (send self :add-controller :rarm-controller)
     (send self :add-controller :head-controller)
     (ros::advertise "/robot/end_effector/right_gripper/command" baxter_core_msgs::EndEffectorCommand 5) 
     (ros::advertise "/robot/end_effector/left_gripper/command" baxter_core_msgs::EndEffectorCommand 5) 
     (ros::advertise "/robot/xdisplay" sensor_msgs::Image 1)
     (ros::advertise "/robot/head/command_head_nod" std_msgs::Bool 1)
     (ros::subscribe "/robot/end_effector/right_gripper/properties" baxter_core_msgs::EndEffectorProperties #'send self :right-property-cb :groupname groupname)
     (ros::subscribe "/robot/end_effector/left_gripper/properties" baxter_core_msgs::EndEffectorProperties #'send self :left-property-cb :groupname groupname)

     (setq right-gripper-action (instance ros::simple-action-client :init
					  "robot/end_effector/right_gripper/gripper_action"
					  control_msgs::GripperCommandAction
					  :groupname groupname))
     (setq left-gripper-action (instance ros::simple-action-client :init
					 "robot/end_effector/left_gripper/gripper_action"
					 control_msgs::GripperCommandAction
					 :groupname groupname
					 ))
     (if (ros::has-param "~wait_for_suction")
	 (setq *wait-for-suction* (read-from-string (ros::get-param "~wait_for_suction"))))
     (dolist (action (list right-gripper-action left-gripper-action))
       (unless (and joint-action-enable (send action :wait-for-server 3))
	 (ros::ros-warn "~A is not respond" action)
	 (ros::ros-info "*** if you do not have gripper, you can ignore this message ***")
	 (return)))

     (setq gripper-sequence-id 0)
     (ros::spin-once)
     (if mvit-rb (setq moveit-robot mvit-rb))
     (if mvit-env (send self :set-moveit-environment (send mvit-env :init :robot moveit-robot)))
     ))
  (:right-property-cb (msg)
   (setq right-gripper-type (send msg :id))
   )
  (:left-property-cb (msg)
   (setq left-gripper-type (send msg :id))
   )
  (:default-controller ()
   (append
    (send self :larm-controller)
    (send self :rarm-controller)
    (send self :head-controller)))
  (:larm-controller ()
   (list
    (list
     (cons :controller-action "/robot/limb/left/follow_joint_trajectory")
     (cons :controller-state "/robot/limb/left/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "left_s0" "left_s1" "left_e0" "left_e1" "left_w0" "left_w1" "left_w2")))))
  (:rarm-controller ()
   (list
    (list
     (cons :controller-action "/robot/limb/right/follow_joint_trajectory")
     (cons :controller-state "/robot/limb/right/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "right_s0" "right_s1" "right_e0" "right_e1" "right_w0" "right_w1" "right_w2")))))
  (:head-controller ()
   (list
    (list
     (cons :controller-action "/robot/head/head_action")
     (cons :controller-state "/robot/head/head_state")
     (cons :action-type control_msgs::SingleJointPositionAction)
     (cons :joint-names (list "head_pan")))))
  (:close-head-camera ()
    (send self :close-camera "head_camera")
    )
  (:close-right-hand-camera ()
    (send self :close-camera "right_hand_camera")
    )
  (:close-left-hand-camera ()
    (send self :close-camera "left_hand_camera")
    )
  (:close-camera (camera)
    (let ((req (instance baxter_core_msgs::CloseCameraRequest :init)))
      (send req :name camera)
      (ros::service-call "/cameras/close" req)
      )
    )
  (:open-head-camera ()
    (send self :open-camera "head_camera")
    )
  (:open-right-hand-camera ()
    (send self :open-camera "right_hand_camera")
    )
  (:open-left-hand-camera ()
    (send self :open-camera "left_hand_camera")
    )
  (:open-camera (camera)
    (let ((camera-setting (instance baxter_core_msgs::CameraSettings :init :width 1280 :height 800 :fps 20))
          (req (instance baxter_core_msgs::OpenCameraRequest :init))
          )
      (send req :name camera)
      (send req :settings camera-setting)
      (ros::service-call "/cameras/open" req)
      )
    )
  (:get-cameras-list()
    (ros::wait-for-service "/cameras/list")
    (let ((req (instance baxter_core_msgs::ListCamerasRequest :init)) res)
      (setq res (ros::service-call "/cameras/list" req))
      (send res :cameras)
      ))

  (:clear-calib-grasp
   (&optional (arm :arms))
   (send self :command-grasp baxter_core_msgs::EndEffectorCommand::*CMD_CLEAR_CALIBRATION* arm))

  (:calib-grasp
   (&optional (arm :arms) &key ((:pos pos) 100))
   (send self :command-grasp baxter_core_msgs::EndEffectorCommand::*cmd_calibrate* arm :pos pos))

  (:start-grasp
   (&optional (arm :arms) &key (effort 50) (wait t))
   (send self :go-grasp arm :pos 0 :effort effort :wait wait)
   (if (or ( and (equal arm :arms) (or (equal right-gripper-type *suction*) (equal left-gripper-type *suction*)))
	   ( and (equal arm :rarm) (equal right-gripper-type *suction*))
	   ( and (equal arm :larm) (equal left-gripper-type *suction*)))
       (unix::usleep *wait-for-suction*)
     )
   )

  (:stop-grasp
   (&optional (arm :arms) &key (effort 50) (wait t))
   (send self :go-grasp arm :pos 100 :effort effort :wait wait)
   (if (or ( and (equal arm :arms) (or (equal right-gripper-type *suction*) (equal left-gripper-type *suction*)))
	   ( and (equal arm :rarm) (equal right-gripper-type *suction*))
	   ( and (equal arm :larm) (equal left-gripper-type *suction*)))
       (unix::usleep *wait-for-suction*)
     )
   )

  (:go-grasp
   (&optional (arm :arms) &key (pos 0) (effort 50) (wait t))
   (let (goal
	 (clients (case arm
		    (:larm (list left-gripper-action))
		    (:rarm (list right-gripper-action))
		    (:arms (list right-gripper-action left-gripper-action))
		    (t (ros::ros-warn "arm[~a] in :move-gripper is invalid type" arm))))
	 (results (list)))
     (dolist (client clients)
       (ros::ros-info "send-goal to ~A" client)
       (setq goal  (instance control_msgs::GripperCommandActionGoal :init))
       (send goal :goal :command :position pos)
       (send goal :goal :command :max_effort effort)
       (send client :send-goal goal)
       )
     
     (if wait (send-all clients :wait-for-result)) 
     (dolist (client clients)
       (push (send client :get-result) results)
       )
     results
     )
   )

  (:command-grasp
   (cmd arm &key ((:pos pos) nil))
   (ros::spin-once)
   (dolist (tmp-arm (if (eq arm :arms) (list :larm :rarm) (list arm)))
     (let ((ee-cmd (instance baxter_core_msgs::EndEffectorCommand :init)))
       (if (eq tmp-arm :rarm)
         (send ee-cmd :id right-gripper-type)
         (send ee-cmd :id left-gripper-type))
       (send ee-cmd :command cmd)
       (send ee-cmd :sender "/baxter_interface")
       (send ee-cmd :sequence gripper-sequence-id)
       (when pos (send ee-cmd :args (format nil "{\"position\": ~A}" pos)))
       (ros::publish (format nil "/robot/end_effector/~A_gripper/command"
                             (if (eq tmp-arm :rarm) "right" "left")) ee-cmd)
       (setq gripper-sequence-id (1+ gripper-sequence-id)))))

  (:set-baxter-face 
   (filepath)
   (let (img msg)
     (setq img (read-jpeg-file filepath))
     
     ;;convert image bgr to rgb
     (dotimes (x (send img :width))
       (dotimes (y (send img :height))
	 (let (r b bd)
	   (setq bd (send img :byte-depth))
	   (setq b (aref img y (* bd x)))
	   (setq r (aref img y (+ (* bd x) 2)))
	   (setf (aref img y (* bd x)) r)
	   (setf (aref img y (+ (* bd x) 2)) b)
	   )
	 )
       )
     (setq msg (instance sensor_msgs::Image :init))
     (send msg :height (send img :height))
     (send msg :width (send img :width))
     (send msg :encoding "rgb8")
     (send msg :step (* (send img :byte-depth) (send img :width)))
     (send msg :data (send img :entity))
     (send msg :header :stamp (ros::time-now))
     (ros::publish "/robot/xdisplay" msg)
     )
   )

  (:nod
   ()
   (let ((msg (instance std_msgs::Bool :init :data t)))
     (ros::publish "/robot/head/command_head_nod" msg)
     )
   )
  (:angle-vector-raw (av &optional (tm :fast) (ctype controller-type) (start-time 0)
                         &key (scale 2.2) (min-time 0.05) (end-coords-interpolation nil)
                         &allow-other-keys)
    (send self :angle-vector-sequence-raw (list av) (list tm) ctype start-time :scale scale :min-time min-time
          :end-coords-interpolation end-coords-interpolation))
  (:angle-vector-sequence-raw (avs &optional (tms :fast) (ctype controller-type) (start-time 0)
                                   &key (scale 2.2) (min-time 0.05) (end-coords-interpolation nil)
                                   &allow-other-keys)
    ;; force add current position to the top of avs
    (if (atom tms) (setq tms (list tms)))
    (setq ctype (or ctype controller-type))  ;; use default if ctype is nil
    (send-super :angle-vector-sequence avs tms ctype start-time :scale scale :min-time min-time
                :end-coords-interpolation end-coords-interpolation))
  (:angle-vector
   (av &optional tm (ctype controller-type) (start-time 0) &rest args
       &key (move-arm :arms) (start-offset-time 0.01) (clear-velocities t) &allow-other-keys)
   "Send joind angle to robot with self-collision motion planning, this method returns immediately, so use :wait-interpolation to block until the motion stops.
- av : joint angle vector [rad]
- tm : time to goal in [msec]
"
   (setq ctype (or ctype controller-type))  ;; use default if ctype is nil
   ;; for simulation mode
   (when (send self :simulation-modep)
     (return-from :angle-vector (send* self :angle-vector-raw av tm ctype start-time args)))
   (if (and (get self :moveit-environment)
            (send (get self :moveit-environment) :robot))
     (progn
       (unless tm (setq tm 3000))
       (send-super* :angle-vector-motion-plan av :ctype ctype :move-arm move-arm :total-time tm
                    :start-offset-time start-offset-time :clear-velocities clear-velocities
                    :use-torso nil args))
     (progn
       (warning-message 3 "moveit environment is not correctly set, execute :angle-vector-raw instead~%")
       (unless tm (setq tm :fast))
       (return-from :angle-vector (send* self :angle-vector-raw av tm ctype start-time args)))))
  (:angle-vector-sequence
   (avs &optional tms (ctype controller-type) (start-time 0) &rest args
        &key (move-arm :arms) (start-offset-time 0.01) (clear-velocities t) &allow-other-keys)
   "Send joind angle sequence to robot with self-collision motion planning, this method returns immediately, so use :wait-interpolation to block until the motion stops.
- avs : sequence of joint angle vector [rad]
- tms : list of time to goal from previous angle-vector point in [msec]
"
   (setq ctype (or ctype controller-type))  ;; use default if ctype is nil
   ;; for simulation mode
   (when (send self :simulation-modep)
     (return-from :angle-vector-sequence
                  (send* self :angle-vector-sequence-raw avs tms ctype start-time args)))
   (if (and (get self :moveit-environment)
            (send (get self :moveit-environment) :robot))
     (progn
       (setq tms
             (if tms
               (if (and (listp tms) (every #'numberp tms)) (apply #'+ tms) tms)
               3000))
       (send-super* :angle-vector-motion-plan avs :ctype ctype :move-arm move-arm :total-time tms
                    :start-offset-time start-offset-time :clear-velocities clear-velocities
                    :use-torso nil args))
     (progn
       (warning-message 3 "moveit environment is not correctly set, execute :angle-vector-sequence-raw instead~%")
       (unless tms (setq tms :fast))
       (return-from :angle-vector-sequence
                    (send* self :angle-vector-sequence-raw avs tms ctype start-time args)))))
  (:ros-state-callback
   (msg)
   (let ((robot-msg-names (send msg :name)) (torso-index))
     ;;Remove toros_t0 . We think this is not the rotational-joint
     (setq torso-index (position "torso_t0" robot-msg-names :test #'string=))
     (send msg :name ( remove "torso_t0" robot-msg-names :test #'string=))
     (when torso-index
       (dolist (key '(:position :velocity :effort) )
         (send msg key (concatenate float-vector 
                                    (subseq (send msg key) 0 torso-index)
                                    (subseq (send msg key) (+ torso-index 1))))))
     ;;End of Removing torso_t0
     (send-super :ros-state-callback msg)
     ))
  ) ;; defclass baxter-interface

(defclass baxter-moveit-environment
  :super moveit-environment)
(defmethod baxter-moveit-environment
  (:init (&key ((:robot rb) *baxter*) &rest args)
         (send-super* :init :robot rb :frame-id "world" args))
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "right_arm")
               (cons :target-link
                     (send self :search-link-from-name "right_gripper"))
               (cons :joint-list (send robot :rarm :joint-list))
               )
         (list :larm
               (cons :group-name "left_arm")
               (cons :target-link
                     (send self :search-link-from-name "left_gripper"))
               (cons :joint-list (send robot :larm :joint-list))
               )
         (list :arms
               ;; can not use inverse-kinematics
               ;; currently only supports angle-vector
               (cons :group-name "both_arms")
               (cons :target-link
                     (list
                       (send self :search-link-from-name "left_gripper")
                       (send self :search-link-from-name "right_gripper")))
               (cons :joint-list (append (send robot :larm :joint-list)
                                         (send robot :rarm :joint-list)))
               )
         )
#| SRDF generated from baxter.srdf.xacro in baxter_moveit_config
   url: https://github.com/ros-planning/moveit_robots/blob/kinetic-devel/baxter/baxter_moveit_config/config/baxter.srdf.xacro

  <group name="left_arm">
    <chain base_link="torso" tip_link="left_gripper"/>
  </group>
  <group name="right_arm">
    <chain base_link="torso" tip_link="right_gripper"/>
  </group>
  <group name="both_arms">
    <group name="right_arm"/>
    <group name="left_arm"/>
  </group>
  <group name="left_hand">
    <chain base_link="left_hand" tip_link="left_gripper"/>
  </group>
  <group name="right_hand">
    <chain base_link="right_hand" tip_link="right_gripper"/>
  </group>
  <!--END EFFECTOR: Purpose: Represent information about an end effector.-->
  <end_effector group="left_hand" name="left_hand_eef" parent_group="left_arm" parent_link="left_gripper"/>
  <end_effector group="right_hand" name="right_hand_eef" parent_group="right_arm" parent_link="right_gripper"/>
  <!--GROUP STATES: Purpose: Define a named state for a particular group, in terms of joint values. This is useful to define states like 'folded arms'-->
  <group_state group="left_arm" name="left_neutral">
    <joint name="left_e0" value="0"/>
    <joint name="left_e1" value="0.75"/>
    <joint name="left_s0" value="0"/>
    <joint name="left_s1" value="-0.55"/>
    <joint name="left_w0" value="0"/>
    <joint name="left_w1" value="1.26"/>
    <joint name="left_w2" value="0"/>
  </group_state>
  <group_state group="right_arm" name="right_neutral">
    <joint name="right_e0" value="0"/>
    <joint name="right_e1" value="0.75"/>
    <joint name="right_s0" value="0"/>
    <joint name="right_s1" value="-0.55"/>
    <joint name="right_w0" value="0"/>
    <joint name="right_w1" value="1.26"/>
    <joint name="right_w2" value="0"/>
  </group_state>
  <!--END EFFECTOR: Purpose: Represent information about an end effector.-->
  <!--VIRTUAL JOINT: Purpose: this element defines a virtual joint between a robot link and an external frame of reference (considered fixed with respect to the robot)-->
  <virtual_joint child_link="torso" name="world_joint" parent_frame="world" type="floating"/>
|#
   ))


(defun baxter-init (&key (safe t) (type :default-controller) (moveit t))
  (let (mvit-env mvit-rb)
    (when moveit
      (setq mvit-env (instance baxter-moveit-environment))
      (setq mvit-rb (instance baxter-robot :init)))
    (if (not (boundp '*ri*))
        (setq *ri* (instance baxter-interface :init :type type
                             :moveit-environment mvit-env
                             :moveit-robot mvit-rb)))
    (if (not (boundp '*baxter*))
        (if safe
        (setq *baxter* (instance baxter-robot-safe :init))
      (setq *baxter* (instance baxter-robot :init))))
    (send *ri* :calib-grasp :arms)))
