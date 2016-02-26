(ros::roseus "pepper")

(load "package://peppereus/pepper.l")
(load "package://naoqieus/naoqi-interface.l")

(defclass pepper-interface
  :super naoqi-interface
  :slots ()
  )

(defmethod pepper-interface
  (:init (&rest args)
   (send-super* :init :robot pepper-robot :naoqi-namespace "pepper_robot" args))
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
  (:servo-on () (call-empty-service "pepper_robot/pose/wakeup"))
  (:servo-off () (call-empty-service "pepper_robot/pose/rest"))
  (:show-image 
   (file)
   (let (ret)
     (ros::wait-for-service "show_image")
     (setq ret (instance naoqi_bridge_msgs::ShowImageRequest :init))
     (send ret :file_name :data file)
     (setq res (ros::service-call "show_image" ret))
     (send res :status :data)
     ))
  (:show-webview 
   (file) 
   (let (ret)
     (ros::wait-for-service "show_webview")
     (setq ret (instance naoqi_bridge_msgs::ShowImageRequest :init))
     (send ret :file_name :data file)
     (setq res (ros::service-call "show_webview" ret))
     (send res :status :data)
     ))
  (:hide-image () (call-empty-service "hide_image"))
  )

(defmethod pepper-robot
  (:reset-pose () (send self :angle-vector #f(2 -2 -5 85 10 -70 -20 -40 85 -10  70  20  40 0 -0)))
  )

(defun pepper-init (&optional (create-viewer))
  (unless (boundp '*pepper*) (pepper))
  (unless (ros::ok) (ros::roseus "pepper_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance pepper-interface :init)))
  
  (ros::spin-once)
  (send *ri* :spin-once)
  
  (send *pepper* :angle-vector (send *ri* :state :potentio-vector))
  (when create-viewer (objects (list *pepper*)))
  )

#|
(setq *ri* (instance pepper-interface :init))
(setq *pepper* (pepper))
(send *pepper* :reset-pose)
(send *ri* :angle-vector (send *pepper* :angle-vector) 2000)
|#
