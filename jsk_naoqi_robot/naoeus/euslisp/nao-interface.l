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
		   nao_msgs::JointTrajectoryAction))
   (ros::advertise (if namespace (format nil "~A/cmd_pose" namespace) "cmd_pose") geometry_msgs::Pose2D 1)
   (ros::advertise (if namespace (format nil "~A/cmd_vel" namespace) "cmd_vel") geometry_msgs::Twist 1)
   
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
     (cons :controller-state "")
     (cons :action-type nao_msgs::JointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))
    ))

  (:add-controller 
   (ctype &key (joint-enable-check))
   (let (tmp-actions)
     (mapcar
      #'(lambda (param)
	  (let* ((controller-action (cdr (assoc :controller-action param)))
		 (action-type (cdr (assoc :action-type param)))
		 (action (instance ros::simple-action-client :init controller-action action-type
				   :groupname groupname)))
	    (push action tmp-actions)))
      (send self ctype))
     (setq tmp-actions (nreverse tmp-actions))
     ;;
     (dolist (action tmp-actions)
       (unless (and joint-action-enable (send action :wait-for-server 3))
	 (ros::ros-warn "~A is not respond, ~A-interface is disabled" action (send robot :name))
	 (when joint-enable-check
	   (setq joint-action-enable nil)
	   (return))))
     
     ;; (dolist (param (send self ctype))
     ;;   (let* ((controller-state (cdr (assoc :controller-state param)))
     ;; 	      (key (intern (string-upcase controller-state) *keyword-package*)))
     ;; 	 (ros::subscribe controller-state
     ;; 			 pr2_controllers_msgs::JointTrajectoryControllerState
     ;; 			 #'send self :set-robot-state1 key :groupname groupname)))
     ;;
     (setf (gethash ctype controller-table) tmp-actions)
     tmp-actions
     ))

  (:servo-on () (send self :send-stiffness-controller 1.0))
  (:servo-off () (send self :send-stiffness-controller 0.0))
  (:send-stiffness-controller
   (stiffness)
   (let ((goal (send joint-stiffness-trajectory-action :make-goal-instance))
	 (joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
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
   )
  (:start-grasp
   (&optional (arm :arms))
   )
  (:stop-grasp
   (&optional (arm :arms))
   )
  (:reset-pose
   ()
   (send self :angle-vector #f(105.292 13.8845 -94.3102 -26.1893 -18.3718 103.363 -15.2077 102.919 29.3582 -30.5008 -2.107 -2.72224 -55.721 121.039 -67.9785 2.02391 -2.107 -7.29261 -55.1105 121.039 -67.97 5.01224 -4.74855 -5.89115) 3000)
   )
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