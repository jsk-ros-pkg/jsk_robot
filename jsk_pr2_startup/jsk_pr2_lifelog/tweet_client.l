#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(ros::roseus "twitter_client")
(ros::advertise "/pr2twit" std_msgs::String 1)

;; next tweet timing ( x(i+1) = x(i) * 2, 5 * 60 = 300 sec)
(while (not (ros::has-param "/active_user/elapsed_time"))
  (unix::sleep 3)
  (ros::ros-info "Wait for /active_user/elapsed_time parameter ..."))
(setq *target-second* (+ (ros::get-param "/active_user/elapsed_time") 300))
(ros::rate 1)
(do-until-key
  (setq *user-name* (ros::get-param "/active_user/user_name")
	*elapsed-time* (ros::get-param "/active_user/elapsed_time"))
  (ros::ros-info "user -> ~A, time -> ~A (~A) " *user-name* *elapsed-time* *target-second*)
  (when (> *elapsed-time* *target-second*)
    (incf *target-second* *target-second*)
    ;; save wide  image and save to /tmp/frame0000.jpg
    (unix::system "(cd /tmp; `rospack find image_view`/bin/image_saver image:=/camera/rgb/image_rect_color _image_transport:=compressed _filename_format:=\"/tmp/frame%04i.jpg\" _sec_per_frame:=1 __name:=image_saver &)")
    (unix::sleep 10)
    (unix::system "rosnode kill image_saver")
    ;; tweet
    (ros::ros-info "tweet ~A ~A" *user-name* *elapsed-time*)
    (ros::publish "/pr2twit"
		  (instance std_msgs::String :init
			    :data (format nil "Congratulations! ~A have reached ~d minutes /tmp/left0000.jpg"
					  (subseq *user-name* 0 (or (position #\space *user-name*) (length *user-name*)))
					  (round (/ *elapsed-time* 60)))))
    (unix::sleep 10)
    (unix::system "rm -f /tmp/left*.jpg")
    )
  (ros::spin-once)
  (ros::sleep)
  )
