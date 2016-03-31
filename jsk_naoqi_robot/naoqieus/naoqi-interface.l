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
  )
;;
