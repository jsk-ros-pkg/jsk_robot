#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(load "package://pr2eus/speak.l")
(ros::roseus "pepper_twitter")


(ros::advertise "/tweet" std_msgs::String 1)
(ros::advertise "/speech" std_msgs::String 1)

(defun speak-jp (str)
  (let ((speech_msg (instance std_msgs::String :init)))
    (send speech_msg :data str)
    (ros::publish "/speech" speech_msg))
  )


(defun tweet (str)
  (let ((timestamp0 0) (timestamp1 0) (filename "/tmp/pepper_camera.png") tm)
    (speak-jp "写真を撮るよ")
    (speak-jp "さん")
    (unix:sleep 1)
    (speak-jp "に")
    (unix:sleep 1)
    (speak-jp "いち")
    (unix:sleep 1)
    (speak-jp "カシャ！")
    (if (probe-file filename)
	(setq timestamp0 (file-write-date filename)))
    (setq timestamp1 timestamp0)
    ;; save image sto /tmp/pepper_camera.jpg
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
    (speak-jp (format nil "~A って ついーとしました" str))
    (ros::ros-info "tweet ~A" str)
    ))

(ros::subscribe "/pepper_tweet" std_msgs::String #'(lambda (msg) (tweet (send msg :data))))
(ros::ros-info "subscribing /pepper_tweet")
(ros::spin)
