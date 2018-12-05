(load "package://pr2eus/robot-interface.l")

;; Treating naoqi_msgs as naoqi_bridge_msgs because ROS hydro uses naoqi_msgs, not naoqi_bridge_msgs
(if (not (find-package "NAOQI_BRIDGE_MSGS"))
    (make-package "NAOQI_BRIDGE_MSGS" :nicknames "NAOQI_MSGS"))

(ros::load-ros-package "naoqi_bridge_msgs")
(ros::load-ros-package "nao_interaction_msgs")

(defclass naoqi-interface
  :super robot-interface
  :slots (naoqi-namespace dcm-namespace joint-stiffness-trajectory-action)
  )

(defmethod naoqi-interface
  (:init
   (&rest args &key ((:naoqi-namespace ns) "naoqi_robot") ((:dcm-namespace dns) "naoqi_dcm") (robot) (type :naoqi-controller) &allow-other-keys)
   (setq naoqi-namespace ns)
   (setq dcm-namespace dns)
   (print args)
   (send-super* :init :robot robot :type type :groupname "naoqi_interface" args)
   (ros::advertise "/move_base_simple/goal" geometry_msgs::PoseStamped 1)
   (ros::advertise "/cmd_vel" geometry_msgs::Twist 1)
   (ros::advertise "/speech" std_msgs::String 1)
   (ros::advertise (format nil "~A/pose/joint_angles" naoqi-namespace) naoqi_bridge_msgs::JointAnglesWithSpeed 1)
   (ros::advertise "/animated_speech" std_msgs::String 1)
   (ros::advertise (format nil "~A/RightHand_controller/command" dcm-namespace) trajectory_msgs::JointTrajectory 1)
   (ros::advertise (format nil "~A/LeftHand_controller/command" dcm-namespace)  trajectory_msgs::JointTrajectory 1)
   (setq joint-stiffness-trajectory-action
	 (instance ros::simple-action-client :init
		   (format nil "~A/pose/joint_stiffness_trajectory" naoqi-namespace)
		   naoqi_bridge_msgs::JointTrajectoryAction))
   self)
  ;;
  (:dcm-head-controller
   ()
   (list
    (list
     (cons :controller-action 
	   (format nil "~A/Head_controller/follow_joint_trajectory" dcm-namespace))
     (cons :controller-state 
	   (format nil "~A/Head_controller/state" dcm-namespace))
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "HeadYaw" "HeadPitch")))))
  (:dcm-larm-controller
   ()
   (list
    (list
     (cons :controller-action 
	   (format nil "~A/LeftArm_controller/follow_joint_trajectory" dcm-namespace))
     (cons :controller-state 
	   (format nil "~A/LeftArm_controller/state" dcm-namespace))
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "LShoulderPitch" "LShoulderRoll" "LElbowYaw" "LElbowRoll" "LWristYaw")))))
  (:dcm-rarm-controller
   ()
   (list
    (list
     (cons :controller-action 
	   (format nil "~A/RightArm_controller/follow_joint_trajectory" dcm-namespace))
     (cons :controller-state 
	   (format nil "~A/RightArm_controller/state" dcm-namespace))
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "RShoulderPitch" "RShoulderRoll" "RElbowYaw" "RElbowRoll" "RWristYaw")))))
  (:dcm-pelvis-controller
   ;; Nao: RHipYawPitch is mimic joint, Pepper: this method is overridden
   ()
   (list
    (list
     (cons :controller-action 
	   (format nil "~A/Pelvis_controller/follow_joint_trajectory" dcm-namespace))
     (cons :controller-state 
	   (format nil "~A/Pelvis_controller/state" dcm-namespace))
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "LHipYawPitch")))))
  (:dcm-lfoot-controller
   ()
   (list
    (list
     (cons :controller-action "/nao_dcm/LeftFoot_controller/follow_joint_trajectory")
     (cons :controller-state "/nao_dcm/LeftFoot_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "LAnklePitch" "LAnkleRoll")))))
  (:dcm-lleg-controller
   ()
   (list
    (list
     (cons :controller-action "/nao_dcm/LeftLeg_controller/follow_joint_trajectory")
     (cons :controller-state "/nao_dcm/LeftLeg_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "LHipRoll" "LHipPitch" "LKneePitch"))))) 
  (:dcm-rfoot-controller
   ()
   (list
    (list
     (cons :controller-action "/nao_dcm/RightFoot_controller/follow_joint_trajectory")
     (cons :controller-state "/nao_dcm/RightFoot_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "RAnklePitch" "RAnkleRoll")))))
  (:dcm-rleg-controller
   ()
   (list
    (list
     (cons :controller-action "/nao_dcm/RightLeg_controller/follow_joint_trajectory")
     (cons :controller-state "/nao_dcm/RightLeg_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "RHipRoll" "RHipPitch" "RKneePitch"))))) 
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
   (if (ros::get-param "use_sim_time" nil)
       (progn
	 (let ((goal (instance trajectory_msgs::JointTrajectory :init)))
	   (setq value (- 1.0 value))
	   (if (or (eq arm :rarm) (eq arm :arms))
	       (progn
		 (send goal :header :seq 1)
		 (send goal :header :stamp (ros::time-now))
		 (send goal :joint_names (list "RHand"))
		 (send goal :points
		       (list (instance trajectory_msgs::JointTrajectoryPoint
				       :init
				       :positions (list value)
				       :time_from_start (ros::time 1))))
		 (ros::publish (format nil "~A/RightHand_controller/command" dcm-namespace) goal)))
	   (if (or (eq arm :larm) (eq arm :arms))
	       (progn
		 (send goal :header :seq 1)
		 (send goal :header :stamp (ros::time-now))
		 (send goal :joint_names (list "LHand"))
		 (send goal :points
		       (list (instance trajectory_msgs::JointTrajectoryPoint
				       :init
				       :positions (list value)
				       :time_from_start (ros::time 1))))
		 (ros::publish (format nil "~A/LeftHand_controller/command" dcm-namespace) goal)))
	   ))
     (progn
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
	 (ros::publish (format nil "~A/pose/joint_angles" naoqi-namespace) start_grasp_msg))))
     )
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
     t))
  (:go-stop
   ()
   (send self :go-velocity 0 0 0 0 :stop nil))
  (:play-audio-file (file)
   (let (ret)
     (ros::wait-for-service "/naoqi_driver/play_audio_file")
     (setq ret (instance nao_interaction_msgs::AudioPlaybackRequest :init))
     (send ret :file_path :data file)
     (setq ret (ros::service-call "/naoqi_driver/play_audio_file" ret))))
  (:set-master-volume 
   (volume)
   (let ((ret (instance nao_interaction_msgs::SetAudioMasterVolumeRequest :init)))
     (ros::wait-for-service "naoqi_driver/set_master_volume")
     (send ret :master_volume :data volume)
     (ros::service-call "/naoqi_driver/set_master_volume" ret)
     ))
  (:get-master-volume
   ()
   (let ((ret (instance nao_interaction_msgs::GetAudioMasterVolumeRequest :init))
	      res)
     (ros::wait-for-service "/naoqi_driver/get_master_volume")
     (setq res (ros::service-call "/naoqi_driver/get_master_volume" ret))
     (send res :master_volume :data)))
  (:enable-life () (call-empty-service (format nil "~A/pose/life/enable" naoqi-namespace)))	
  (:disable-life () (call-empty-service (format nil "~A/pose/life/disable" naoqi-namespace)))
  (:get-life ()
	     (ros::wait-for-service (format nil "~A/pose/life/get_state" naoqi-namespace))
	     (setq req (instance std_srvs::TriggerRequest :init))
	     (setq res (ros::service-call (format nil "~A/pose/life/get_state" naoqi-namespace) req))
	     (send res :message)
	     )
  (:set-external-collision-protection-status 
   (type status)
   (let (req res)
     (ros::wait-for-service "/set_external_collision_protection_status")
     (setq req (instance naoqi_bridge_msgs::SetExternalCollisionProtectionEnabledRequest :init))
     (send req :name (instance naoqi_bridge_msgs::ExternalCollisionProtectionNames :init :data type))
     (send req :status status)
     (setq res (ros::service-call "/set_external_collision_protection_status" req))
     (send res :success)))
  (:get-external-collision-protection-status 
   (type)
   (let (req res)
     (ros::wait-for-service "/get_external_collision_protection_status")
     (setq req (instance naoqi_bridge_msgs::GetExternalCollisionProtectionEnabledRequest :init))
     (send req :name (instance naoqi_bridge_msgs::ExternalCollisionProtectionNames :init :data type))
     (setq res (ros::service-call "/get_external_collision_protection_status" req))
     (send res :status)
     ))
  (:set-move-arms-enabled 
   (status &optional (arm :arms))
   (let ((ret (instance naoqi_bridge_msgs::SetArmsEnabledRequest :init))
	 (another-arm-status))
     (ros::wait-for-service "/set_move_arms_enabled")
     (case arm
	   (:arms 
	    (send ret :left_arm status)
	    (send ret :right_arm status))
	   (:rarm
	    (setq another-arm-status (send *ri* :get-move-arms-enabled 0))
	    (send ret :right_arm status)
	    (send ret :left_arm another-arm-status))
	   (:larm 
	    (setq another-arm-status (send *ri* :get-move-arms-enabled 1))
	    (send ret :right_arm another-arm-status)
	    (send ret :left_arm status)))
     (ros::service-call "/set_move_arms_enabled" ret))
   )
  (:get-move-arms-enabled
   (&optional (arm :arms))
   (let ((ret (instance naoqi_bridge_msgs::GetArmsEnabledRequest :init))
	 (arm_msg (instance naoqi_bridge_msgs::Arms :init)) res)
     (ros::wait-for-service "/get_move_arms_enabled")
     (case arm
	   (:larm 
	    (send arm_msg :data 0))
	   (:rarm 
	    (send arm_msg :data 1))
	   (:arms 
	    (send arm_msg :data 2)))
     (send ret :arm arm_msg)
     (setq res (ros::service-call "/get_move_arms_enabled" ret))
     (send res :status))
   )
  (:set-language
   (language)
   (let ((ret (instance naoqi_bridge_msgs::SetStringRequest :init))
	 res)
     (ros::wait-for-service "/naoqi_driver/set_language")
     (send ret :data language)
     (setq res (ros::service-call "/naoqi_driver/set_language" ret))
     (send res :success))
   )
  (:get-language
   ()
   (let ((ret (instance naoqi_bridge_msgs::GetStringRequest :init))
	 res)
     (ros::wait-for-service "/naoqi_driver/get_language")
     (setq res (ros::service-call "/naoqi_driver/get_language" ret))
     (send res :data))
   )
  (:set-background-movement-enabled
   (status)
   (let ((ret (instance std_srvs::SetBoolRequest :init))
	  res)
     (ros::wait-for-service "/background_movement/set_enabled")
     (send ret :data status)
     (setq res (ros::service-call "/background_movement/set_enabled" ret))
     (send res :success)
     ))
  (:get-background-movement-enabled
   ()
   (let ((ret (instance std_srvs::TriggerRequest :init))
	 res)
     (ros::wait-for-service "/background_movement/is_enabled")
     (setq res (ros::service-call "/background_movement/is_enabled" ret))
     (send res :success)
     )
   )
  (:fade-leds
   (led_name r g b sec)
   (let* ((fade_rgb_msg (instance naoqi_bridge_msgs::FadeRGB :init))
	 (ret (instance naoqi_bridge_msgs::FadeLedsRequest :init))
	 (duration_msg (send fade_rgb_msg :fade_duration))
	 res)
     (ros::wait-for-service "/naoqi_driver/fade_leds")
     (send fade_rgb_msg :led_name led_name)
     (send fade_rgb_msg :color :r r)
     (send fade_rgb_msg :color :g g)
     (send fade_rgb_msg :color :b b)
     (send duration_msg :sec sec)
     (send fade_rgb_msg :fade_duration duration_msg)
     (send ret :fade_rgb fade_rgb_msg)
     (setq res (ros::service-call "/naoqi_driver/fade_leds" ret))
     )
   )
  (:reset-leds
   (led_name)
   (let ((ret (instance naoqi_bridge_msgs::SetStringRequest :init))
	 res)
     (send ret :data led_name)
     (setq res (ros::service-call "/naoqi_driver/reset_leds" ret))
     (send res :success)
     )
   )
  )
;;
