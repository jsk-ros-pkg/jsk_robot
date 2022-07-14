(ros::roseus "pepper")

(require :pepper "package://peppereus/pepper.l")
(load "package://naoqieus/naoqi-interface.l")

(defclass pepper-interface
  :super naoqi-interface
  :slots ()
  )

(defmethod pepper-interface
  (:init (&rest args &key ((:group-namespace gns) ""))
   (send-super* :init :robot pepper-robot :group-namespace gns :naoqi-namespace "pepper_robot" :dcm-namespace "pepper_dcm" args)
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
	 (cons :controller-action (format nil "~A/~A/pose/joint_trajectory" group-namespace naoqi-namespace))
	 ;;(cons :controller-state "joint_trajectory")
	 (cons :controller-state "dummy_state") ;; this is dummy
	 (cons :action-type naoqi_bridge_msgs::JointTrajectoryAction)
	 (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name)))))
       )))
  (:servo-on
   ()
   (call-empty-service (format nil "~A/~A/pose/wakeup" group-namespace naoqi-namespace)))
  (:servo-off
   ()
   (call-empty-service (format nil "~A/~A/pose/rest" group-namespace naoqi-namespace)))
  (:show-image
   (file)
   (let (ret)
     (ros::wait-for-service (format nil "~A/show_image" group-namespace))
     (setq ret (instance naoqi_bridge_msgs::SetStringRequest :init))
     (send ret :data file)
     (setq res (ros::service-call (format nil "~A/show_image" group-namespace) ret))
     (send res :success)
     ))
  (:set-show-image-folder-path
   (name)
   (let (ret)
     (ros::wait-for-service (format nil "~A/set_show_image_folder_path" group-namespace))
     (setq ret (instance naoqi_bridge_msgs::SetStringRequest :init))
     (send ret :data name)
     (setq res (ros::service-call (format nil "~A/set_show_image_folder_path" group-namespace) ret))
     (send res :success)
     ))
  (:get-show-image-folder-path
   ()
   (let (ret)
     (ros::wait-for-service (format nil "~A/get_show_image_folder_path" group-namespace))
     (setq ret (instance naoqi_bridge_msgs::GetStringRequest :init))
     (setq res (ros::service-call (format nil "~A/get_show_image_folder_path" group-namespace) ret))
     (send res :data)
     ))
  (:show-app
   (app)
   (let (ret)
     (ros::wait-for-service (format nil "~A/show_app" group-namespace))
     (setq ret (instance naoqi_bridge_msgs::SetStringRequest :init))
     (send ret :data app)
     (setq res (ros::service-call (format nil "~A/show_app" group-namespace) ret))
     (send res :success)
     ))
  (:show-webview
   (url)
   (let (ret)
     (ros::wait-for-service (format nil "~A/show_webview" group-namespace))
     (setq ret (instance naoqi_bridge_msgs::SetStringRequest :init))
     (send ret :data url)
     (setq res (ros::service-call (format nil "~A/show_webview" group-namespace) ret))
     (send res :success)
     ))
  (:hide-image () (call-empty-service (format nil "~A/hide_image" group-namespace)))
  )

(defmethod pepper-robot
  (:reset-pose () (send self :angle-vector #f(2 -2 -5 85 10 -70 -20 -40 85 -10  70  20  40 0 -0)))
  )

(defun pepper-init (&optional (create-viewer) (group-namespace ""))
  (unless (boundp '*pepper*) (pepper))
  (unless (ros::ok) (ros::roseus "pepper_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance pepper-interface :init :group-namespace group-namespace)))

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
