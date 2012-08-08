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
    ;; save wide  image and save to /tmp/frame0000.jpg
    (unix::system "`rospack find image_view`/bin/extract_images image:=/wide_stereo/right/image_rect_color _image_transport:=compressed _filename_format:=\"/tmp/frame%04i.jpg\" _sec_per_frame:=1 __name:=extract_images &")
    (unix::sleep 4)
    (unix::system "rosnode kill extract_images")
    ;; tweet
    (ros::ros-info "tweet ~A ~A" *user-name* *elapsed-time*)
    (ros::publish "/pr2twit"
		  (instance std_msgs::String :init
			    :data (format nil "Congratulations! ~A have reached ~d minutes /tmp/frame0000.jpg"
					  (subseq *user-name* 0 (or (position #\space *user-name*) (length *user-name*)))
					  (round (/ *elapsed-time* 60))))))
  (ros::spin-once)
  (ros::sleep)
  )
