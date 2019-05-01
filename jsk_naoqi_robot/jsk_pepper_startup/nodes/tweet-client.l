#!/usr/bin/env roseus

(ros::roseus "pepper_tweet")
(ros::load-ros-package "peppereus")

(ros::advertise "/pepper_tweet" std_msgs::String 1)

(defun tweet (str &rest args)
  (ros::publish "/pepper_tweet" (instance std_msgs::String :init :data (apply #'format nil str args))))

(progn (unix:sleep 3) (tweet "おはよう") (ros::spin))
