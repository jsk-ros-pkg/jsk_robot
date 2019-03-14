(ros::roseus "pepper")

(load "package://peppereus/pepper.l")
(load "package://naoqieus/naoqi-interface.l")

(defclass pepper-interface
  :super naoqi-interface
  :slots ()
  )

(defmethod pepper-interface
  (:init (&rest args)
   (send-super* :init :robot pepper-robot :naoqi-namespace "pepper_robot" :dcm-namespace "pepper_dcm" args)
   (when (ros::get-param "use_sim_time" nil)
     ;; add controllers for gazebo
     (dolist (l (list
		 (cons :dcm-head-controller "/pepper_dcm/Head_controller/follow_joint_trajectory")
		 (cons :dcm-larm-controller "/pepper_dcm/LeftArm_controller/follow_joint_trajectory")
		 (cons :dcm-lhand-controller "/pepper_dcm/LeftHand_controller/follow_joint_trajectory")
		 (cons :dcm-pelvis-controller "/pepper_dcm/Pelvis_controller/follow_joint_trajectory")
		 (cons :dcm-rarm-controller "/pepper_dcm/RightArm_controller/follow_joint_trajectory")
		 (cons :dcm-rhand-controller "/pepper_dcm/RightHand_controller/follow_joint_trajectory")))
       (let ((type (car l))
	     (name (cdr l))
	     action)
	 (setq action (find-if #'(lambda (ac) (string= name (send ac :name)))
			       controller-actions))
	 (setf (gethash type controller-table) (list action))
	 )))
   )
  (:dcm-pelvis-controller
   ()
   (list
    (list
     (cons :controller-action "/pepper_dcm/Pelvis_controller/follow_joint_trajectory")
     (cons :controller-state "/pepper_dcm/Pelvis_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "HipRoll" "HipPitch" "KneePitch")))))
  (:naoqi-controller
   ()
   (if (ros::get-param "use_sim_time" nil)
       (progn
	 ;; only for simulation
	 (append
	  (send self :dcm-head-controller)
	  (send self :dcm-larm-controller)
	  (send self :dcm-pelvis-controller)
	  (send self :dcm-rarm-controller)
	  )
	 )
     (progn
       (list
	(list
	 (cons :controller-action (format nil "~A/pose/joint_trajectory" naoqi-namespace))
	 ;;(cons :controller-state "joint_trajectory")
	 (cons :controller-state "dummy_state") ;; this is dummy
	 (cons :action-type naoqi_bridge_msgs::JointTrajectoryAction)
	 (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name)))))
       )))
  (:show-image 
   (file)
   (let (ret)
     (ros::wait-for-service "show_image")
     (setq ret (instance naoqi_bridge_msgs::SetStringRequest :init))
     (send ret :data file)
     (setq res (ros::service-call "show_image" ret))
     (send res :success)
     ))
  (:set-show-image-folder-path 
   (name)
   (let (ret)
     (ros::wait-for-service "set_show_image_folder_path")
     (setq ret (instance naoqi_bridge_msgs::SetStringRequest :init))
     (send ret :data name)
     (setq res (ros::service-call "set_show_image_folder_path" ret))
     (send res :success)
     ))
  (:get-show-image-folder-path 
   ()
   (let (ret)
     (ros::wait-for-service "get_show_image_folder_path")
     (setq ret (instance naoqi_bridge_msgs::GetStringRequest :init))
     (setq res (ros::service-call "get_show_image_folder_path" ret))
     (send res :data)
     ))			       
  (:show-app 
   (app)
   (let (ret)
     (ros::wait-for-service "show_app")
     (setq ret (instance naoqi_bridge_msgs::SetStringRequest :init))
     (send ret :data app)
     (setq res (ros::service-call "show_app" ret))
     (send res :success)
     ))
  (:show-webview 
   (url) 
   (let (ret)
     (ros::wait-for-service "show_webview")
     (setq ret (instance naoqi_bridge_msgs::SetStringRequest :init))
     (send ret :data url)
     (setq res (ros::service-call "show_webview" ret))
     (send res :success)
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
