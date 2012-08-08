#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(ros::roseus "twitter_client")
(ros::advertise "/pr2twit" std_msgs::String 1)

;; tweet evely 30 miniutes (= 30 * 60 = 1800 sec)
(setq *target-second* (+ (ros::get-param "/active_user/elapsed_time") 1800))
(ros::rate 1)
(do-until-key
  (setq *user-name* (ros::get-param "/active_user/user_name")
	*elapsed-time* (ros::get-param "/active_user/elapsed_time"))
  (ros::ros-info "user -> ~A, time -> ~A (~A) " *user-name* *elapsed-time* *target-second*)
  (when (> *elapsed-time* *target-second*)
    (incf *target-second* 1800)
    (ros::ros-info "tweet ~A ~A" *user-name* *elapsed-time*)
    (ros::publish "/pr2twit"
		  (instance std_msgs::String :init
			    :data (format nil "Congratulations! ~A have reached ~d minutes"
					  (subseq *user-name* 0 (or (position #\space *user-name*) (length *user-name*)))
					  (round (/ *elapsed-time* 60))))))
  (ros::spin-once)
  (ros::sleep)
  )
