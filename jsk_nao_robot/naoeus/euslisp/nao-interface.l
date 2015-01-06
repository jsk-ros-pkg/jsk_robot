(ros::roseus "nao")

(load "package://naoeus/euslisp/nao.l")
(load "package://pr2eus/robot-interface.l")
(ros::load-ros-manifest "naoeus")

(defclass nao-interface
  :super robot-interface
  :slots (joint-stiffness-trajectory-action)
  )

(defmethod nao-interface
  (:init
   ;;   (&rest args &key (type :default-controller) &allow-other-keys)
   (&rest args &key (type :nao-controller) ((:namespace ns)) &allow-other-keys)
   ;; wait for pr2-action server (except move_base)

   (send-super* :init :robot nao-robot :type type :groupname (format nil "~A/nao_interface" ns) :namespace ns args)

   (setq joint-stiffness-trajectory-action
	 (instance ros::simple-action-client :init
		   (if namespace (format nil "~A/joint_stiffness_trajectory" namespace ) "joint_stiffness_trajectory")
		   naoqi_msgs::JointTrajectoryAction))
   (ros::advertise (if namespace (format nil "~A/cmd_pose" namespace) "cmd_pose") geometry_msgs::Pose2D 1)
   (ros::advertise (if namespace (format nil "~A/cmd_vel" namespace) "cmd_vel") geometry_msgs::Twist 1)
   (ros::advertise "/speech" std_msgs::String 1)
   (ros::advertise "/joint_angles" naoqi_msgs::JointAnglesWithSpeed 1)
   ;; (dolist (param (send self controller-type))
   ;;   (let* ((controller-state (cdr (assoc :controller-state param)))
   ;; 	    (key (intern (string-upcase controller-state) *keyword-package*)))
   ;;     (when controller-state
   ;; 	 (ros::subscribe controller-state
   ;; 			 nao_msgs::JointTrajectoryResult
   ;; 			 #'send self :set-robot-state1 key))
   ;;   ))
   )

  (:nao-controller
   ()
   (list
    (list
     (cons :controller-action (if namespace (format nil "~A/joint_trajectory" namespace ) "joint_trajectory"))
     ;;"joint_trajectory")
     ;;(cons :controller-state "joint_trajectory")
     (cons :controller-state "dummy_state") ;; this is dummy
     (cons :action-type naoqi_msgs::JointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))
    ))

  (:servo-on () (send self :send-stiffness-controller 1.0))
  (:servo-off () (send self :send-stiffness-controller 0.0))
  (:send-stiffness-controller
   (stiffness)
   (let ((goal (send joint-stiffness-trajectory-action :make-goal-instance))
	 (joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (setq joint-names (append joint-names (list "RHand" "LHand")))
     (send goal :goal :trajectory :joint_names joint-names)
     (send goal :goal :trajectory :header :stamp (ros::time-now))
     (send goal :goal :trajectory :points
	   (list (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions (fill (instantiate float-vector (length joint-names)) stiffness)
			   :time_from_start (ros::time 1))))
     (send joint-stiffness-trajectory-action :send-goal goal)
     ))
  (:get-stiffness
   ()
   )

  (:go-pos
   (x y &optional (d 0)) ;; [m] [m] [degree]
   (let ((pose_msg (instance geometry_msgs::Pose2D :init)))
     (send pose_msg :x x)
     (send pose_msg :y y)
     (send pose_msg :theta (deg2rad d))
     (ros::publish (if namespace (format nil "~A/cmd_pose" namespace) "cmd_pose") pose_msg)
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
     (ros::publish (if namespace (format nil "~A/cmd_vel" namespace) "cmd_vel") vel_msg)
     (when stop
       (unix:usleep (* 1000 msec))
       (send self :go-stop)
       )
     ))
  (:go-stop
   ()
   (send self :go-velocity 0 0 0 0 :stop nil)
   )
  
  (:speak
   (str)
   (let ((speech_msg (instance std_msgs::String :init)))
     (send speech_msg :data str) 
     (ros::publish "/speech" speech_msg))
   )
  
  (:start-grasp
   (&optional (arm :arms))
   (let ((start_grasp_msg (instance naoqi_msgs::JointAnglesWithSpeed :init)))
     (send start_grasp_msg :header :stamp (ros::time-now))
     (send start_grasp_msg :header :seq 1)
     (send start_grasp_msg :speed 0.5)
     (send start_grasp_msg :relative 0)
     (case arm  
       (:arms
	(send start_grasp_msg :joint_names (list "RHand" "LHand"))
	(send start_grasp_msg :joint_angles (list 0.0 0.0)))
       (:rarm 
	(send start_grasp_msg :joint_names (list "RHand"))
	(send start_grasp_msg :joint_angles (list 0.0 )))
       (:larm 
	(send start_grasp_msg :joint_names (list "LHand"))
	(send start_grasp_msg :joint_angles (list 0.0))))
     (ros::publish "/joint_angles" start_grasp_msg)
     ))
  (:stop-grasp
   (&optional (arm :arms))
   (let ((stop_grasp_msg (instance naoqi_msgs::JointAnglesWithSpeed :init)))
     (send stop_grasp_msg :header :stamp (ros::time-now))
     (send stop_grasp_msg :header :seq 1)
     (send stop_grasp_msg :speed 0.5)
     (send stop_grasp_msg :relative 0)
     (case arm  
       (:arms
	(send stop_grasp_msg :joint_names (list "RHand" "LHand"))
	(send stop_grasp_msg :joint_angles (list 1.0 1.0)))
       (:rarm 
	(send stop_grasp_msg :joint_names (list "RHand"))
	(send stop_grasp_msg :joint_angles (list 1.0 )))
       (:larm 
	(send stop_grasp_msg :joint_names (list "LHand"))
	(send stop_grasp_msg :joint_angles (list 1.0))))
     (ros::publish "/joint_angles" stop_grasp_msg)
     ))
  (:reset-pose
   ()
   (send self :angle-vector #f(105.292 13.8845 -94.3102 -26.1893 -18.3718 103.363 -15.2077 102.919 29.3582 -30.5008 -2.107 -2.72224 -55.721 121.039 -67.9785 2.02391 -2.107 -7.29261 -55.1105 121.039 -67.97 5.01224 -4.74855 -5.89115) 3000)
   )
  )

(defun nao-init (&optional (create-viewer))
  (unless (boundp '*nao*) (nao))
  (unless (ros::ok) (ros::roseus "nao_eus_interface"))
  (unless (boundp '*ni*) (setq *ni* (instance nao-interface :init)))
  
  (ros::spin-once)
  (send *ni* :spin-once)
  
  (send *nao* :angle-vector (send *ni* :state :potentio-vector))
  (when create-viewer (objects (list *nao*)))
  )

#|
(setq *nao* (nao))
(objects (list *nao*))
(setq *group-ns* "nao")
(if (boundp '*group-ns*)
    (setq *ni* (instance nao-interface :init :namespace *group-ns*))
  (setq *ni* (instance nao-interface :init))
  )
(unix:sleep 1)
(send *ni* :servo-on)
(dotimes (i 3)
  (send *nao* :arms :elbow-p :joint-angle (if (evenp i) -20 -90))
  (send *ni* :angle-vector (send *nao* :angle-vector) 2000)
  ;;(send *ni* :wait-interpolation)
  )

(send *ni* :go-pos 1 0 0)
|#