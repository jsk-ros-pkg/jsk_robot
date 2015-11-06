#!/usr/bin/env roseus


(ros::roseus "joy_client")
(ros::load-ros-manifest "peppereus")


(if (not (find-package "NAOQI_BRIDGE_MSGS"))
    (make-package "NAOQI_BRIDGE_MSGS" :nicknames "NAOQI_MSGS"))

(ros::advertise "/speech" std_msgs::String 1)
(defun speak-jp (str &rest args)
  (let ((msg (instance std_msgs::String :init)))
    (send msg :data (apply #'format nil str args))
    (ros::ros-info "speech ~A" (send msg :data))
    (ros::publish "/speech" msg))
  )

(ros::advertise "/pepper_tweet" std_msgs::String 1)
(defun tweet (str &rest args)
  (ros::publish "/pepper_tweet" (instance std_msgs::String :init :data (apply #'format nil str args))))


(ros::advertise "/pepper_robot/pose/joint_angles" naoqi_bridge_msgs::JointAnglesWithSpeed 1)

(setq *power-button-time* (ros::time-now))
(setq *power-button-mode* "/pepper_robot/pose/wakeup")
(setq *power-button-state* 0)
(ros::subscribe "/joy" sensor_msgs::Joy
		#'(lambda (msg)
		    (let* ((buttons (send msg :buttons))
			   (axes (send msg :axes))
			   (power-button-state (elt buttons 8))
			   (rb-button-state (elt buttons 5))
			   (b-button-state (elt buttons 1))
			   (x-button-state (elt buttons 2))
			   (y-button-state (elt buttons 3)))
		      (print (list buttons power-button-state (ros::time- (ros::time-now) *power-button-time*)))
		      (cond ((and (= *power-button-state* 1)
				  (=  power-button-state  0)
				  (> (send (ros::time- (ros::time-now) *power-button-time*) :to-sec) 2))
			     (if (string= *power-button-mode* "/pepper_robot/pose/wakeup")
				 (progn
				   (speak-jp "もう朝かな")
				   (call-empty-service *power-button-mode*)
				   (tweet "おはよう")
				   (setq *power-button-mode* "/pepper_robot/pose/rest"))
			       (progn
				 (speak-jp "もうそろそろ寝ます")
				 (call-empty-service *power-button-mode*)
				 (tweet "おやすみ")
				 (setq *power-button-mode* "/pepper_robot/pose/wakeup")))
			     (unix:sleep 5)
			     );; when
			    ((= rb-button-state 1)
			     (let ((up-down (elt axes 4))
				   (left-right (elt axes 3))
				   (ja_msg (instance naoqi_bridge_msgs::JointAnglesWithSpeed :init)))
			       (send ja_msg :header :stamp (ros::time-now))
			       (send ja_msg :header :seq 1)
			       (send ja_msg :speed 0.1)
			       (send ja_msg :relative 1)
			       (send ja_msg :joint_names (list "HeadYaw" "HeadPitch"))
			       (send ja_msg :joint_angles (scale 0.1 (float-vector left-right up-down)))
			       (ros::publish "/pepper_robot/pose/joint_angles" ja_msg)
			       ))
			    ((= b-button-state 1)
			     (speak-jp "disabled")
			     (call-empty-service "/pepper_robot/pose/life/disable")
			     (unix:sleep 1)
			     (call-empty-service "/pepper_robot/pose/wakeup"))
			    ((= x-button-state 1)
			     (speak-jp "interactive")
			     (call-empty-service "/pepper_robot/pose/life/enable"))
			    (t
			     ))
		      (setq *power-button-time* (ros::time-now))
		      (setq *power-button-state* power-button-state)
		      )));;


(ros::ros-info "start joy-client.l")
(unix:sleep 1) ;; need to wait 1 sec to speek
(speak-jp "joy client")
(tweet "おはよう")
(ros::spin)


