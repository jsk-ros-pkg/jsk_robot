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
  (speak-jp "写真を撮るよ")
  (speak-jp "さん")
  (unix:sleep 1)
  (speak-jp "に")
  (unix:sleep 1)
  (speak-jp "いち")
  (unix:sleep 1)
  (speak-jp "カシャ！")
  ;; save image sto /tmp/pepper_camera.jpg
  (ros::service-call "/image_saver/save" (instance std_srvs::EmptyRequest :init))
  (ros::sleep 3)
  (ros::ros-info "save image")
  ;; tweet
  (ros::publish "/tweet" (instance std_msgs::String :init :data (format nil "~A /tmp/pepper_camera.png" str)))
  (speak-jp (format nil "~A って ついーとしました" str))
  (ros::ros-info "tweet ~A" str)
  )

(ros::subscribe "/pepper_tweet" std_msgs::String #'(lambda (msg) (tweet (send msg :data))))
(ros::spin)
