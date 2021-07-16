#!/usr/bin/env roseus

(ros::load-ros-package "roseus")
(ros::load-ros-package "naoqi_bridge_msgs")
(ros::roseus "pepper_twitter_server")

(ros::advertise "/tweet" std_msgs::String 1)
(ros::advertise "/speech" std_msgs::String 1)
(ros::advertise "/pepper_robot/pose/joint_angles" naoqi_bridge_msgs::JointAnglesWithSpeed 1)

(defun get-language ()
  (let ((ret (instance naoqi_bridge_msgs::GetStringRequest :init))
	res)
    (ros::wait-for-service "/naoqi_driver/get_language")
    (setq res (ros::service-call "/naoqi_driver/get_language" ret))
    (send res :data)
    ))

(defun speak (str)
  (let ((speech_msg (instance std_msgs::String :init)))
    (send speech_msg :data str)
    (ros::publish "/speech" speech_msg))
  )

(defun take-pose (position_list)
  (let ((joint-trajectory-action (instance ros::simple-action-client :init
					   "/pepper_robot/pose/joint_trajectory" naoqi_bridge_msgs::JointTrajectoryAction
					   ))
	goal
	trajectory)
    (if (send joint-trajectory-action :wait-for-server 3)
	(progn
	  (setq goal (send joint-trajectory-action :make-goal-instance))
	  (setq trajectory (send goal :goal :trajectory))
	  (send trajectory :joint_names (list "KneePitch" "HipRoll" "HipPitch" "LShoulderPitch" "LShoulderRoll" "LElbowYaw" "LElbowRoll" "LWristYaw" "RShoulderPitch" "RShoulderRoll" "RElbowYaw" "RElbowRoll" "RWristYaw" "HeadYaw" "HeadPitch"))
	  (send trajectory :points
		(list (instance trajectory_msgs::JointTrajectoryPoint
				:init
				:positions position_list
				:time_from_start (ros::time 1.5))))
	  (send joint-trajectory-action :send-goal goal)
	  (send joint-trajectory-action :wait-for-result)
	  (send joint-trajectory-action :get-result))
      (ros::ros-warn "No joint trajectory action found")
      )))

(defun move-rhand (value)
  (let ((move_hand_msg (instance naoqi_bridge_msgs::JointAnglesWithSpeed :init)))
    (send move_hand_msg :header :stamp (ros::time-now))
    (send move_hand_msg :header :seq 1)
    (send move_hand_msg :speed 0.5)
    (send move_hand_msg :relative 0)
    (send move_hand_msg :joint_names (list "RHand"))
    (send move_hand_msg :joint_angles (list value))
    (ros::publish "/pepper_robot/pose/joint_angles" move_hand_msg)))

(defun tweet-without-image (str)
  (let (speech-language)
    (setq speech-language (get-language))
    ;; tweet
    (ros::publish "/tweet" (instance std_msgs::String :init :data (format nil "~A" str)))
    (if (string= speech-language "Japanese")
	(speak (format nil "\\vct=120\\~A\\pau=500\\ってツイートしました" str))
      (speak (format nil "\\pau=1000\\\\rspd=90\\\\vct=120\\I tweeted \\pau=500\\ ~A\\rspd=100\\" str)))
    (ros::ros-info "tweet ~A" str)
    ))

(defun tweet-with-image (str)
  (let (speech-language (timestamp0 0) (timestamp1 0) (filename "/tmp/pepper_camera.png") tm)
    (setq speech-language (get-language))
    ;; taking picture pose
    (take-pose (mapcar #'deg2rad (list 2.0 -2.0 -5.0 100.0 50.0 -20.0 -80.0 -40.0 -60.0 -20.0 -10.0 60.0 40.0 0.0 0.0)))
    (if (string= speech-language "Japanese")
	(progn
	  (speak "\\vct=120\\写真を撮るよぉー")
	  (speak "\\vct=120\\さんっ")
	  (unix:sleep 2)
	  (speak "\\vct=120\\にっ")
	  (unix:sleep 2)
	  (speak "\\vct=120\\いちっ")
	  (unix:sleep 2)
	  (speak"\\vct=120\\カしゃっ！")
	  )
      (progn
	(speak "\\rspd=90\\\\vct=120\\I'll take a picture")
	(unix:sleep 2)
	(speak "\\rspd=90\\\\vct=120\\three")
	(unix:sleep 2)
	(speak "\\rspd=90\\\\vct=120\\two")
	(unix:sleep 2)
	(speak "\\rspd=90\\\\vct=120\\one")
	(unix:sleep 2)
	(speak "\\rspd=90\\\\vct=120\\ka sha")
	))
    ;; close hand
    (move-rhand 0.0)
    (unix:sleep 1)
    ;; take reset pose
    (take-pose (mapcar #'deg2rad (list 2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
    ;; open hand
    (move-rhand 1.0)
    (if (probe-file filename)
	(setq timestamp0 (file-write-date filename)))
    (setq timestamp1 timestamp0)
    ;; save image to /tmp/pepper_camera.png
    (ros::service-call "/image_saver/save" (instance std_srvs::EmptyRequest :init))
    (setq tm (ros::time-now))
    (ros::ros-info "~A timestamp ~A ~A" filename timestamp0 timestamp1)
    (ros::rate 2)
    (while (and (< (send (ros::time- (ros::time-now) tm) :to-sec) 10)
		(= timestamp0 timestamp1))
      (if (probe-file filename) (setq timestamp1 (file-write-date filename)))
      (ros::ros-info "~A timestamp ~A ~A" filename timestamp0 timestamp1)
      (ros::sleep))
    (ros::ros-info "save image")
    ;; tweet
    (ros::publish "/tweet" (instance std_msgs::String :init :data (format nil "~A ~A" str filename)))
    (if (string= speech-language "Japanese")
	(speak (format nil "\\vct=120\\~A\\pau=500\\ってツイートしました" str))
      (speak (format nil "\\pau=1000\\\\rspd=90\\\\vct=120\\I tweeted \\pau=500\\ ~A\\rspd=100\\" str)))
    (ros::ros-info "tweet ~A" str)
    ))

(ros::subscribe "/pepper_tweet" std_msgs::String #'(lambda (msg) (tweet-with-image (send msg :data))))
(ros::subscribe "/pepper_tweet_without_image" std_msgs::String #'(lambda (msg) (tweet-without-image (send msg :data))))
(ros::ros-info "subscribing /pepper_tweet")
(ros::spin)
(exit)
