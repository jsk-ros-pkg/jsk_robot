(ros::roseus "nao")

(load "package://naoeus/nao.l")
(load "package://naoqieus/naoqi-interface.l")

(defclass nao-interface
  :super robot-interface
  :slots (joint-stiffness-trajectory-action)
  )

(defclass nao-interface
  :super naoqi-interface
  :slots ()
  )

(defmethod nao-interface
  (:init (&rest args)
   (send-super* :init :robot nao-robot :naoqi-namespace "nao_robot" args))
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
  (:servo-on () (call-empty-service "nao_robot/pose/wakeup"))
  (:servo-off () (call-empty-service "nao_robot/pose/rest"))
  )

;; current nao version H25V50
(defclass nao-robot
  :super naoH25V50-robot)
(defun nao () (setq *nao* (naoH25V50)))

(defmethod nao-robot
  (:reset-pose () (send self :angle-vector #f(110.0 10.0 -90.0 -40.0 0.0   110.0 -10.0 90.0 40.0 0.0  0.0 0.0 -30.0 60.0 -30.0 0.0   0.0 0.0 -30.0 60.0 -30.0 0.0  0.0 0.0)))
  )

(defun nao-init (&optional (create-viewer))
  (unless (boundp '*nao*) (nao))
  (unless (ros::ok) (ros::roseus "nao_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance nao-interface :init)))
  
  (ros::spin-once)
  (send *ri* :spin-once)
  
  (send *nao* :angle-vector (send *ri* :state :potentio-vector))
  (when create-viewer (objects (list *nao*)))
  )

#|
(setq *ri* (instance nao-interface :init))
(setq *nao* (nao))
(send *nao* :reset-pose)
(send *ri* :angle-vector (send *nao* :angle-vector) 2000)
|#
