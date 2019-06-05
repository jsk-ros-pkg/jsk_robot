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
  (:init (&rest args &key ((:group-namespace gns) ""))
   (send-super* :init :robot nao-robot :group-namespace gns :naoqi-namespace "nao_robot" :dcm-namespace "nao_dcm" args)
   (when (ros::get-param "use_sim_time" nil)
     ;; add controllers for gazebo
     (dolist (l (list
		 (cons :dcm-head-controller "/nao_dcm/Head_controller/follow_joint_trajectory")
		 (cons :dcm-larm-controller "/nao_dcm/LeftArm_controller/follow_joint_trajectory")
		 (cons :dcm-lfoot-controller "/nao_dcm/LeftFoot_controller/follow_joint_trajectory")
		 (cons :dcm-lleg-controller "/nao_dcm/LeftLeg_controller/follow_joint_trajectory")
		 (cons :dcm-lhand-controller "/nao_dcm/LeftHand_controller/follow_joint_trajectory")
		 (cons :dcm-pelvis-controller "/nao_dcm/Pelvis_controller/follow_joint_trajectory")
		 (cons :dcm-rarm-controller "/nao_dcm/RightArm_controller/follow_joint_trajectory")
		 (cons :dcm-rfoot-controller "/nao_dcm/RightFoot_controller/follow_joint_trajectory")
		 (cons :dcm-rleg-controller "/nao_dcm/RightLeg_controller/follow_joint_trajectory")
		 (cons :dcm-rhand-controller "/nao_dcm/RightHand_controller/follow_joint_trajectory")))
       (let ((type (car l))
	     (name (cdr l))
	     action)
	 (setq action (find-if #'(lambda (ac) (string= name (send ac :name)))
			       controller-actions))
	 (setf (gethash type controller-table) (list action))
	 )))
   )
  (:naoqi-controller
   ()
   (if (ros::get-param "use_sim_time" nil)
       (progn
	 ;; only for simulation
	 (append
	  (send self :dcm-head-controller)
	  (send self :dcm-larm-controller)
	  (send self :dcm-lfoot-controller)
	  (send self :dcm-lleg-controller)
	  (send self :dcm-pelvis-controller)
	  (send self :dcm-rarm-controller)
	  (send self :dcm-rfoot-controller)
	  (send self :dcm-rleg-controller)
	  ))
     (progn
       (list
	(list
	 (cons :controller-action (format nil "~A/~A/pose/joint_trajectory" group-namespace naoqi-namespace))
	 ;;(cons :controller-state "joint_trajectory")
	 (cons :controller-state "dummy_state") ;; this is dummy
	 (cons :action-type naoqi_bridge_msgs::JointTrajectoryAction)
	 (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))
	)))
   )
  (:servo-on
   ()
   (call-empty-service (format nil "~A/~A/pose/wakeup" group-namespace naoqi-namespace)))
  (:servo-off
   ()
   (call-empty-service (format nil "~A/~A/pose/rest" group-namespace naoqi-namespace)))
  )
;; current nao version H25V50
(defclass nao-robot
  :super naoH25V50-robot)
(defun nao () (setq *nao* (naoH25V50)))

(defmethod nao-robot
  (:reset-pose () (send self :angle-vector #f(110.0 10.0 -90.0 -40.0 0.0   110.0 -10.0 90.0 40.0 0.0  0.0 0.0 -30.0 60.0 -30.0 0.0   0.0 0.0 -30.0 60.0 -30.0 0.0  0.0 0.0)))
  )

(defun nao-init (&optional (create-viewer) (group-namespace ""))
  (unless (boundp '*nao*) (nao))
  (unless (ros::ok) (ros::roseus "nao_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance nao-interface :init :group-namespace group-namespace)))

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
