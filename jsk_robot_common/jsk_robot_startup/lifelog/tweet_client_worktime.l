#!/usr/bin/env roseus

(ros::roseus "twitter_client_worktime")

(require :tweet_client "package://jsk_robot_startup/lifelog/tweet_client.l")

(defvar *robot-name* (ros::get-param "/robot/name" "robot"))
(defvar *user-name* (ros::get-param "/active_user/launch_user_name" nil))
(defvar *worktime-tweet-interval* (ros::get-param "~worktime_tweet_interval" 300.0))
(defvar *prev-elapsed-time* 0.0)

(defun tweet-worktime (&optional event)
  (unless (ros::has-param "/active_user/elapsed_time")
    (return-from tweet-worktime nil))
  (unless *user-name*
    (setq *user-name* (ros::get-param "/active_user/launch_user_name" nil)))
  (let ((elapsed-time (ros::get-param "/active_user/elapsed_time"))
        mainstr presubstr postsubstr)
    (ros::ros-info "tweet ~A ~A" *user-name* elapsed-time)
    (if (eq elapsed-time *prev-elapsed-time*)
        (return-from tweet-worktime t)
        (setq *prev-elapsed-time* elapsed-time))
    (setq mainstr (format nil "~A has used ~A for ~d minutes"
                          (or *user-name* "An user") *robot-name* (round (/ elapsed-time 60))))
    (cond
      ((< elapsed-time 600) ;; 5 min
       (setq presubstr "Congratulations! "
             postsubstr ", Let's get started!"))
      ((< elapsed-time 910) ;; 15 min
       (setq presubstr "Gooood! "
             postsubstr ", Go ahead!"))
      ((< elapsed-time 1820) ;; 30 min
       (setq presubstr "So Nice! "
              postsubstr ", Go ahead!"))
      ((< elapsed-time 2730) ;; 45 min
       (setq presubstr "Fantastic! "
             postsubstr ", Keep going!"))
      ((< elapsed-time 3640) ;; 60 min
       (setq presubstr "Amazing! "
             postsubstr ", I'm not tired!"))
      (t
       (setq presubstr "Awesome! "
             postsubstr ", Got some rest?")))

    (tweet-string (format nil "~A~A~A" presubstr mainstr postsubstr)
                  :wait-for-camera 3 :with-image t)))

(defun main ()
  (ros::create-timer *worktime-tweet-interval* #'tweet-worktime)
  (ros::spin))

(main)
