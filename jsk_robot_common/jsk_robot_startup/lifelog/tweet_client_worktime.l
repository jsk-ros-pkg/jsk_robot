#!/usr/bin/env roseus

(ros::roseus "twitter_client_worktime")

(load "package://jsk_robot_startup/lifelog/tweet_client.l")

(defvar *robot-name* "robot")
(when (ros::has-param "/active_user/robot_name")
  (setq *robot-name* (ros::get-param "/active_user/robot_name"))
  )

;; next tweet timing ( x(i+1) = x(i) * 2, 5 * 60 = 300 sec)
(while (not (ros::has-param "/active_user/elapsed_time"))
  (unix::sleep 3)
  (ros::ros-info "Wait for /active_user/elapsed_time parameter ..."))

(cond
 ((ros::has-param "/active_user/tweet_second")
  (setq *tweet-second* (ros::get-param "/active_user/tweet_second")))
 (t
  (setq *tweet-second* 300)
  ))

(setq *target-second* (+ (ros::get-param "/active_user/elapsed_time")
                         *tweet-second*))

(setq *waking-tweet-second* 3600.0)
(cond
 ((ros::has-param "/active_user/start_time")
  (let ((st (ros::get-param "/active_user/start_time")))
    (setq *waking-target-second*
          (+ (- (send (ros::time-now) :to-sec) st)
             *waking-tweet-second*))))
 (t
  (setq *waking-target-second* *waking-tweet-second*)))

(ros::advertise "/tweet" std_msgs::String 1)
(ros::rate 0.1)
(do-until-key
  (setq *user-name* (ros::get-param "/active_user/launch_user_name")
        *elapsed-time* (ros::get-param "/active_user/elapsed_time"))
  (ros::ros-debug "user -> ~A, time -> ~A (~A) "
                 *user-name* *elapsed-time* *target-second*)
  ;; tweet depend on working time
  (when (> *elapsed-time* *target-second*)
    (incf *target-second* *tweet-second* )
    (ros::ros-info "tweet ~A ~A" *user-name* *elapsed-time*)
    (let ((mainstr (format nil "~A has used ~A for ~d minutes"
                           ;; why delete *user-name* after space ?
                           ;;(subseq *user-name* 0
                           ;;(or (position #\space *user-name*)
                           ;;(length *user-name*)))
                           *user-name*
			   *robot-name*
                           (round (/ *elapsed-time* 60))))
          presubstr postsubstr)
      (cond
       ((< *elapsed-time* 600) ;; 5 min
        (setq presubstr "Congratulations! "
              postsubstr ", Let's get started!"))
       ((< *elapsed-time* 910) ;; 15 min
        (setq presubstr "Gooood! "
              postsubstr ", Go ahead!"))
       ((< *elapsed-time* 1820) ;; 30 min
        (setq presubstr "So Nice! "
              postsubstr ", Go ahead!"))
       ((< *elapsed-time* 2730) ;; 45 min
        (setq presubstr "Fantastic! "
              postsubstr ", Keep going!"))
       ((< *elapsed-time* 3640) ;; 60 min
        (setq presubstr "Amazing! "
              postsubstr ", I'm not tired!"))
       (t
        (setq presubstr "Awesome! "
              postsubstr ", Got some rest?")))

      (tweet-string (format nil "~A~A~A" presubstr mainstr postsubstr)
                    :warning-time 1 :with-image t)

      ))
  (ros::sleep)
  )
