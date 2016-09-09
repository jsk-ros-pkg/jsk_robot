(load "package://pr2eus/robot-interface.l")
(ros::load-ros-manifest "naoqieus")

(if (not (find-package "NAOQI_BRIDGE_MSGS"))
    (make-package "NAOQI_BRIDGE_MSGS" :nicknames "NAOQI_MSGS"))

(defclass naoqi-interface
  :super robot-interface
  :slots (naoqi-namespace joint-stiffness-trajectory-action)
  )

(defmethod naoqi-interface
  (:init
   (&rest args &key ((:naoqi-namespace ns) "naoqi_robot") (robot) (type :naoqi-controller) &allow-other-keys)
   (setq naoqi-namespace ns)
   (print args)
   (send-super* :init :robot robot :type type :groupname "naoqi_interface" args)
   (ros::advertise "/move_base_simple/goal" geometry_msgs::PoseStamped 1)
   (ros::advertise "/cmd_vel" geometry_msgs::Twist 1)
   (ros::advertise "/speech" std_msgs::String 1)
   (ros::advertise (format nil "~A/pose/joint_angles" naoqi-namespace) naoqi_bridge_msgs::JointAnglesWithSpeed 1)
   (ros::advertise "/animated_speech" std_msgs::String 1)
   (setq joint-stiffness-trajectory-action
	 (instance ros::simple-action-client :init
		   (format nil "~A/pose/joint_stiffness_trajectory" naoqi-namespace)
		   naoqi_bridge_msgs::JointTrajectoryAction))
   (ros::advertise "/pepper_dcm/Head_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/pepper_dcm/LeftArm_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/pepper_dcm/LeftHand_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/pepper_dcm/Pelvis_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/pepper_dcm/RightArm_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/pepper_dcm/RightHand_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/pepper_dcm/Wheels_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/nao_dcm/Head_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/nao_dcm/LeftArm_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/nao_dcm/LeftFoot_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/nao_dcm/LeftHand_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/nao_dcm/LeftLeg_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/nao_dcm/Pelvis_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/nao_dcm/RightArm_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/nao_dcm/RightFoot_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/nao_dcm/RightHand_controller/command" trajectory_msgs::JointTrajectory 1)
   (ros::advertise "/nao_dcm/RightLeg_controller/command" trajectory_msgs::JointTrajectory 1)
   self)
  ;;
  (:naoqi-controller
   ()
   (list
    (list
     (cons :controller-action (format nil "~A/pose/joint_trajectory" naoqi-namespace))
    ;;(cons :controller-state "joint_trajectory")
     (cons :controller-state "dummy_state") ;; this is dummy
     (cons :action-type naoqi_bridge_msgs::JointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))
    ))
  ;;
  (:angle-vector
   (av &optional (tm nil) (ctype controller-type) (start-time 0) &key (scale 1) (min-time 1.0))
   "Send joind angle to robot, this method retuns immediately, so use :wait-interpolation to block until the motion stops.
- av : joint angle vector [rad]
- tm : (time to goal in [msec])
   if designated tm is faster than fastest speed, use fastest speed
   if not specified, it will use 1/scale of the fastest speed .
   if :fastest is specefied use fastest speed calcurated from max speed
- ctype : controller method name
- start-time : time to start moving
- scale : if tm is not specified, it will use 1/scale of the fastest speed
- min-time : minimum time to for time to goal
"
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
      ;;Normal Number disgnated Mode
      ((numberp tm)
       (if (< tm fastest-tm)
           (setq tm fastest-tm)))
      ;;Safe Mode (Speed will be 5 * fastest-time)
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
   ;; for gazebo mode
   (when (send *ri* :robot-interface-simulation-callback) 
     (if (boundp '*pepper*) 
	 (progn
	   (let* ((angle-list (send *pepper* :angle-vector)))
	     (dotimes (i (length angle-list)) (setf (elt angle-list i) (deg2rad (elt angle-list i))))
	     (send *ri* :gazebo-head-controller (list (elt angle-list 13) (elt angle-list 14))) 
	     (send *ri* :gazebo-left-arm-controller (list (elt angle-list 3) (elt angle-list 4) (elt angle-list 5) (elt angle-list 6) (elt angle-list 7)))
	     (send *ri* :gazebo-pelvis-controller (list (elt angle-list 1) (elt angle-list 2) (elt angle-list 0)))
	     (send *ri* :gazebo-right-arm-controller (list (elt angle-list 8) (elt angle-list 9) (elt angle-list 10) (elt angle-list 11) (elt angle-list 12)))
	     ;;(send *ri* :gazebo-left-hand-controller (list ))
	     ;;(send *ri* :gazebo-right-hand-controller (list ))
	     ;;(send *ri* :gazebo-wheels-controller (list ))
	     )))
     (if (boundp '*nao*) 
	 (progn
	   (let* ((angle-list (send *nao* :angle-vector)))
	     (dotimes (i (length angle-list)) (setf (elt angle-list i) (deg2rad (elt angle-list i))))
	     (send *ri* :gazebo-head-controller (list (elt angle-list 22) (elt angle-list 23))) 
	     (send *ri* :gazebo-left-arm-controller (list (elt angle-list 0) (elt angle-list 1) (elt angle-list 2) (elt angle-list 3) (elt angle-list 4)))
	     (send *ri* :gazebo-right-arm-controller (list (elt angle-list 5) (elt angle-list 6) (elt angle-list 7) (elt angle-list 8) (elt angle-list 9)))
	     (send *ri* :gazebo-left-foot-controller (list (elt angle-list 14) (elt angle-list 15)))
	     (send *ri* :gazebo-left-leg-controller (list (elt angle-list 11) (elt angle-list 12) (elt angle-list 13)))
	     (send *ri* :gazebo-right-foot-controller (list (elt angle-list 20) (elt angle-list 21)))
	     (send *ri* :gazebo-right-leg-controller (list (elt angle-list 17) (elt angle-list 18) (elt angle-list 19)))
	     (send *ri* :gazebo-pelvis-controller (list (elt angle-list 10)))
	     ;;(send *ri* :gazebo-left-hand-controller (list ))
	     ;;(send *ri* :gazebo-right-hand-controller (list ))
	     )))
     )
   (send robot :angle-vector av)
   (let ((cacts (gethash ctype controller-table)))
     (mapcar
      #'(lambda (action param)
	  (send self :send-ros-controller
		action (cdr (assoc :joint-names param)) ;; action server and joint-names
		start-time  ;; start time
		(list
		 (list av                                     ;; positions
		       (instantiate float-vector (length av)) ;; velocities
		       (/ tm 1000.0)))))                      ;; duration
      cacts (send self ctype)))
   av)
  ;;
  (:error-vector () (map float-vector #'rad2deg (send self :state :effort)))
  ;;
  (:send-stiffness-controller
   (joint  stiffness)
   (let ((goal (send joint-stiffness-trajectory-action :make-goal-instance))
	 joint-name)
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (case joint  
       (:larm
	(setq joint-name (list "LArm")))
       (:rarm 
	(setq joint-name (list "RArm")))
       (:head
	(setq joint-name (list "Head")))
       (:lhand
	(setq joint-name (list "LHand")))
       (:rhand
	(setq joint-name (list "RHand")))
       )
     (send goal :goal :trajectory :joint_names joint-name)
     (send goal :goal :trajectory :header :stamp (ros::time-now))
     (send goal :goal :trajectory :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions (fill (instantiate float-vector (length joint-name)) stiffness)
			   :time_from_start (ros::time 1))))
     (send joint-stiffness-trajectory-action :send-goal goal)
     ))
  (:servo-on () 
	     (if (boundp '*nao*) 
		 (call-empty-service "nao_robot/pose/wakeup"))
	     (if (boundp '*pepper*)
		 (call-empty-service "pepper_robot/pose/wakeup")))
	     
  (:servo-off () 
	      (if (boundp '*nao*) 
		 (call-empty-service "nao_robot/pose/rest"))
	      (if (boundp '*pepper*)
		 (call-empty-service "pepper_robot/pose/rest")))
  ;;
  (:move-hand
   (value &optional (arm :arms))
   (let ((start_grasp_msg (instance naoqi_bridge_msgs::JointAnglesWithSpeed :init)))
     (send start_grasp_msg :header :stamp (ros::time-now))
     (send start_grasp_msg :header :seq 1)
     (send start_grasp_msg :speed 0.5)
     (send start_grasp_msg :relative 0)
     (case arm  
       (:arms
	(send start_grasp_msg :joint_names (list "RHand" "LHand"))
	(send start_grasp_msg :joint_angles (list value value)))
       (:rarm 
	(send start_grasp_msg :joint_names (list "RHand"))
	(send start_grasp_msg :joint_angles (list value)))
       (:larm 
	(send start_grasp_msg :joint_names (list "LHand"))
	(send start_grasp_msg :joint_angles (list value))))
     (ros::publish (format nil "~A/pose/joint_angles" naoqi-namespace) start_grasp_msg)
     ))
  (:start-grasp
   (&optional (angle-ratio 0.0) (arm :arms))
   (if (memq angle-ratio '(:larm :rarm :arms))
       (if (numberp arm)
	   (progn
	     (setq tmp arm)
	     (setq arm angle-ratio)
	     (setq angle-ratio tmp))
	 (progn
	   (setq arm angle-ratio)
	   (setq angle-ratio 0.0))))
   (if (not (memq arm '(:larm :rarm :arms))) (format t "~A is invalid. Allowable parameters are :rarm, :larm, :arms. " arm))
   (cond ((> angle-ratio 0.5) 
	  (format t "~A is invalid. 0.5 is adapted. (The range is between 0.0 and 0.5) " angle-ratio)
	  (setq angle-ratio 0.5)
	  )
	 ((< angle-ratio 0.0) 
	  (format t "~A is invalid. 0.0 is adapted. (The range is between 0.0 and 1.0) " angle-ratio)
	  (setq angle-ratio 0.0)
	  ))
   (send self :move-hand angle-ratio arm)
   )
  (:stop-grasp
   (&optional (angle-ratio 1.0) (arm :arms))
   (if (memq angle-ratio '(:larm :rarm :arms))
       (if (numberp arm)
	   (progn
	     (setq tmp arm)
	     (setq arm angle-ratio)
	     (setq angle-ratio tmp))
	 (progn
	   (setq arm angle-ratio)
	   (setq angle-ratio 1.0))))
   (if (not (memq arm '(:larm :rarm :arms))) (format t "~A is invalid. Allowable parameters are :rarm, :larm, :arms. " arm))
   (cond ((> angle-ratio 1.0) 
	  (format t "~A is invalid. 1.0 is adapted. (The range is between 0.5 and 1.0) " angle-ratio)
	  (setq angle-ratio 1.0)
	  )
	 ((< angle-ratio 0.5) 
	  (format t "~A is invalid. 0.5 is adapted. (The range is between 0.5 and 1.0) " angle-ratio)
	  (setq angle-ratio 0.5)
	  ))
   (send self :move-hand angle-ratio arm)
   )
  ;;
  (:speak
   (str)
   (let ((speech_msg (instance std_msgs::String :init)))
     (send speech_msg :data str)
     (ros::publish "/speech" speech_msg))
   )
  (:animated-speak
   (str)
   (let ((animated_speech_msg (instance std_msgs::String :init)))
     (send animated_speech_msg :data str)
     (ros::publish "/animated_speech" animated_speech_msg))
   )
  ;;
  (:go-pos
   (x y &optional (d 0)) ;; [m] [m] [degree]
   "move robot toward x, y, degree and wait to reach that goal. return t if reached or nil if fail
    the robot moves relative to current position.
    using [m] and [degree] is historical reason from original hrpsys code"
   ;; https://github.com/jsk-ros-pkg/jsk_pr2eus/blob/master/pr2eus/robot-interface.l#L867
   (let (c (pose_msg (instance geometry_msgs::PoseStamped :init)))
     (setq c (make-coords :pos (float-vector (* 1000 x) (* y 1000) 0)
			  :rpy (float-vector (deg2rad d) 0 0)))
     (send pose_msg :header :frame_id "base_footprint")
     (send pose_msg :pose (ros::coords->tf-pose c))
     (ros::publish "/move_base_simple/goal" pose_msg)
     ))

  (:go-velocity
   (x y d &optional (msec 1000) &key (stop t)) ;; [m/sec] [m/sec] [rad/sec]
   (let ((vel_msg (instance geometry_msgs::Twist :init)))
     (when (> (abs x) 1)
       (ros::ros-error "x must be in range [-1 <-> 1]")
       (return-from :go-velocity nil))
     (when (> (abs y) 1)
       (ros::ros-error "y must be in range [-1 <-> 1]")
       (return-from :go-velocity nil))
     (when (> (abs d) 1)
       (ros::ros-error "theta must be in range [-1 <-> 1]")
       (return-from :go-velocity nil))
     (send vel_msg :linear :x x)
     (send vel_msg :linear :y y)
     (send vel_msg :angular :z d)
     (ros::publish "/cmd_vel" vel_msg)
     (when stop
       (unix:usleep (* 1000 msec))
       (send self :go-stop)
       )
     ))
  (:go-stop
   ()
   (send self :go-velocity 0 0 0 0 :stop nil))
  (:play-audio-file (file)
   (let (ret)
     (ros::wait-for-service "nao_audio/play_file")
     (setq ret (instance nao_interaction_msgs::AudioPlaybackRequest :init))
     (send ret :file_path :data file)
     (setq ret (ros::service-call "nao_audio/play_file" ret))))
  (:set-master-volume (volume)
   (let (ret)
     (ros::wait-for-service "nao_audio/master_volume")
     (setq ret (instance nao_interaction_msgs::AudioMasterVolumeRequest :init))
     (send ret :master_volume :data volume)
     (setq ret (ros::service-call "nao_audio/master_volume" ret))))
  (:enable-life () (call-empty-service (format nil "~A/pose/life/enable" naoqi-namespace)))	
  (:disable-life () (call-empty-service (format nil "~A/pose/life/disable" naoqi-namespace)))
  (:get-life ()
	     (ros::wait-for-service (format nil "~A/pose/life/getState" naoqi-namespace))
	     (setq req (instance naoqi_bridge_msgs::GetAlifeStateRequest :init))
	     (setq res (ros::service-call (format nil "~A/pose/life/getState" naoqi-namespace) req))
	     (send res :status :data)
	     )
  (:gazebo-head-controller 
   (angle-list)
   (let ((goal (instance trajectory_msgs::JointTrajectory :init))
	 joint-name)
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (setq joint-name (list "HeadYaw" "HeadPitch")) 
     (send goal :joint_names joint-name)
     (send goal :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions angle-list
			   :time_from_start (ros::time 1))))
     (if (boundp '*nao*) 
	 (ros::publish "/nao_dcm/Head_controller/command" goal))
     (if (boundp '*pepper*) 
	 (ros::publish "/pepper_dcm/Head_controller/command" goal))
     ))
  (:gazebo-left-arm-controller 
   (angle-list)
   (let ((goal (instance trajectory_msgs::JointTrajectory :init))
	 joint-name)
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (setq joint-name (list "LShoulderPitch" "LShoulderRoll" "LElbowYaw" "LElbowRoll" "LWristYaw"))
     (send goal :joint_names joint-name)
     (send goal :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions angle-list
			   :time_from_start (ros::time 1))))
     (if (boundp '*nao*) 
	 (ros::publish "/nao_dcm/LeftArm_controller/command" goal))
     (if (boundp '*pepper*)
	 (ros::publish "/pepper_dcm/LeftArm_controller/command" goal))
     ))
  (:gazebo-left-hand-controller 
   (angle-list)
   (let ((goal (instance trajectory_msgs::JointTrajectory :init))
	 joint-name)
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (setq joint-name (list "LHand")) 
     (send goal :joint_names joint-name)
     (send goal :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions angle-list
			   :time_from_start (ros::time 1))))
     (if (boundp '*nao*) 
	 (ros::publish "/nao_dcm/LeftHand_controller/command" goal))
     (if (boundp '*pepper*)
	 (ros::publish "/pepper_dcm/LeftHand_controller/command" goal))
     ))
  (:gazebo-pelvis-controller 
   (angle-list)
   (let ((goal (instance trajectory_msgs::JointTrajectory :init))
	 joint-name)     
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (if (boundp '*nao*)
	 (setq joint-name (list "LHipYawPitch")))       
     (if (boundp `*pepper*)
	 (setq joint-name (list "HipRoll" "HipPitch" "KneePitch")))
     (send goal :joint_names joint-name)
     (send goal :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions angle-list
			   :time_from_start (ros::time 1))))
     (if (boundp '*nao*)
	 (ros::publish "/nao_dcm/Pelvis_controller/command" goal))
     (if (boundp '*pepper*)
	 (ros::publish "/pepper_dcm/Pelvis_controller/command" goal))
     ))
  (:gazebo-right-arm-controller 
   (angle-list)
   (let ((goal (instance trajectory_msgs::JointTrajectory :init))
	 joint-name)
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (setq joint-name (list "RShoulderPitch" "RShoulderRoll" "RElbowYaw" "RElbowRoll" "RWristYaw")) 
     (send goal :joint_names joint-name)
     (send goal :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions angle-list
			   :time_from_start (ros::time 1))))
     (if (boundp '*nao*)
	 (ros::publish "/nao_dcm/RightArm_controller/command" goal))
     (if (boundp '*pepper*)
	 (ros::publish "/pepper_dcm/RightArm_controller/command" goal))
     ))
  (:gazebo-right-hand-controller 
   (angle-list)
   (let ((goal (instance trajectory_msgs::JointTrajectory :init))
	 joint-name)
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (setq joint-name (list "RHand")) 
     (send goal :joint_names joint-name)
     (send goal :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions angle-list
			   :time_from_start (ros::time 1))))
     (if (boundp '*nao*)
	 (ros::publish "/nao_dcm/RightHand_controller/command" goal))
     (if (boundp '*pepper*)
	 (ros::publish "/pepper_dcm/RightHand_controller/command" goal))
     ))
  (:gazebo-wheels-controller 
   (angle-list)
   (let ((goal (instance trajectory_msgs::JointTrajectory :init))
	 joint-name)
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (setq joint-name (list "WheelFL" "WheelB" "WheelFR")) 
     (send goal :joint_names joint-name)
     (send goal :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions angle-list
			   :time_from_start (ros::time 1))))
     (ros::publish "/pepper_dcm/Wheels_controller/command" goal)
     ))
  (:gazebo-left-foot-controller 
   (angle-list)
   (let ((goal (instance trajectory_msgs::JointTrajectory :init))
	 joint-name)
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (setq joint-name (list "LAnklePitch" "LAnkleRoll")) 
     (send goal :joint_names joint-name)
     (send goal :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions angle-list
			   :time_from_start (ros::time 1))))
     (ros::publish "/nao_dcm/LeftFoot_controller/command" goal)
     ))
  (:gazebo-left-leg-controller 
   (angle-list)
   (let ((goal (instance trajectory_msgs::JointTrajectory :init))
	 joint-name)
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (setq joint-name (list "LHipRoll" "LHipPitch" "LKneePitch")) 
     (send goal :joint_names joint-name)
     (send goal :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions angle-list
			   :time_from_start (ros::time 1))))
     (ros::publish "/nao_dcm/LeftLeg_controller/command" goal)
     ))
  (:gazebo-right-foot-controller 
   (angle-list)
   (let ((goal (instance trajectory_msgs::JointTrajectory :init))
	 joint-name)
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (setq joint-name (list "RAnklePitch" "RAnkleRoll")) 
     (send goal :joint_names joint-name)
     (send goal :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions angle-list
			   :time_from_start (ros::time 1))))
     (ros::publish "/nao_dcm/RightFoot_controller/command" goal)
     ))
  (:gazebo-right-leg-controller 
   (angle-list)
   (let ((goal (instance trajectory_msgs::JointTrajectory :init))
	 joint-name)
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (setq joint-name (list "RHipRoll" "RHipPitch" "RKneePitch")) 
     (send goal :joint_names joint-name)
     (send goal :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions angle-list
			   :time_from_start (ros::time 1))))
     (ros::publish "/nao_dcm/RightLeg_controller/command" goal)
     ))
  )
;;
