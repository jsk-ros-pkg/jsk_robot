#!/usr/bin/env roseus
(defvar *camera-topic-name* "/openni/rgb/image_rect_color")
(ros::load-ros-manifest "roseus")

(ros::roseus "twitter_client")
(ros::advertise "/pr2twit" std_msgs::String 1)

;; next tweet timing ( x(i+1) = x(i) * 2, 5 * 60 = 300 sec)
(while (not (ros::has-param "/active_user/elapsed_time"))
  (unix::sleep 3)
  (ros::ros-info "Wait for /active_user/elapsed_time parameter ..."))

(cond
 ((ros::has-param "/active_user/tweet_second")
  (setq *tweet-second* (ros::get-param "/active_user/tweet_second")))
 (t
  (setq *tweet-second* 300)))
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

(ros::rate 1)
(do-until-key
  (setq *user-name* (ros::get-param "/active_user/user_name")
        *elapsed-time* (ros::get-param "/active_user/elapsed_time"))
  (ros::ros-info "user -> ~A, time -> ~A (~A) "
                 *user-name* *elapsed-time* *target-second*)
  ;; tweet depend on up time
  (let ((st (ros::get-param "/active_user/start_time")))
    (when st
      (let ((waking-time (- (send (ros::time-now) :to-sec) st)))
        (ros::ros-info "PR2 waking ~A sec (~A)" waking-time *waking-target-second*)
        (when (> waking-time *waking-target-second*)
          (incf *waking-target-second* *waking-tweet-second*)
          ;;(ros::ros-info "waking tweet !!")
          (let ((fname (format nil "/tmp/tweet_~d.txt" (unix::getpid)))
                str
                lines)
            (unix::system
             (format nil "fortune hitchhiker -n 274 -s > ~A" fname))
            (with-open-file
             (f fname)
             (while (setq str (read-line f nil nil))
               (push str lines)))
            (let* ((ln (apply #'+ (length lines)
                              (mapcar #'(lambda (x) (length x)) lines)))
                   (dt (make-string (1- ln)))
                   (pos 0))
              (dolist (s (nreverse lines))
                (replace dt s :start1 pos)
                (incf pos (length s))
                (if (< pos (- ln 2))
                    (setf (elt dt pos) 10))
                (incf pos))
              ;;(print dt)
              (ros::ros-info "waking tweet !!")
              (ros::publish "/pr2twit"
                            (instance std_msgs::String :init :data dt))
              ))
          ))))
  ;; tweet depend on working time
  (when (> *elapsed-time* *target-second*)
    (incf *target-second* *tweet-second* )
    ;; save wide  image and save to /tmp/frame0000.jpg
    (unix::system "rm -f /tmp/left*.jpg")
    (unix::system
     (format nil "(cd /tmp; `rospack find image_view`/bin/image_saver image:=~A _image_transport:=compressed _filename_format:=\"/tmp/frame%04i.jpg\" _sec_per_frame:=1 __name:=image_saver &)"
             *camera-topic-name*))
    (unix::sleep 10)
    (unix::system "rosnode kill image_saver")
    ;; tweet
    (ros::ros-info "tweet ~A ~A" *user-name* *elapsed-time*)
    (let ((mainstr (format nil "~A have used PR2 ~d minutes"
                           (subseq *user-name* 0
                                   (or (position #\space *user-name*)
                                       (length *user-name*)))
                           (round (/ *elapsed-time* 60))))
          (filenamestr "/tmp/left0000.jpg")
          presubstr postsubstr)
      (cond
       ((< *elasped-time* 600) ;; 10 min
        (setq presubstr "Congratulations! "
              postsubstr ", Let's get started!"))
       ((> *elasped-time* 900) ;; 15 min
        (setq presubstr "Gooood! "
              postsubstr ", Go ahead!"))
       ((> *elasped-time* 1800) ;; 30min
        (setq presubstr "Fantastic! "
              postsubstr ", Keep going!"))
       ((> *elasped-time* 2700) ;; 45min
        (setq presubstr "Amazing! "
              postsubstr ", I'm not tired!"))
       ((> *elasped-time* 3600) ;; 60min
        (setq presubstr "Awesome! "
              postsubstr ", Got some rest?")))

      (ros::publish "/pr2twit"
                    (instance std_msgs::String :init
                              :data (format nil "~A~A~A ~A"
                                            presubstr
                                            mainstr
                                            postsubstr
                                            filenamestr)))
      ))
  (ros::spin-once)
  (ros::sleep)
  )
