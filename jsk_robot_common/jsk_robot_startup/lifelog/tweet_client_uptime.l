#!/usr/bin/env roseus

(ros::roseus "twitter_client_uptime")

(load "package://roseus/euslisp/dynamic-reconfigure-server.l")
(load "package://jsk_robot_startup/lifelog/tweet_client.l")

(setq *src-lines* nil)
(setq *random-state* (coerce (unix::gettimeofday) integer-vector))

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

(setq *volume* (ros::get-param "~volume" 1.0))
(setq *speak-enable* (ros::get-param "~speak_enable" t))
(setq *enable* (ros::get-param "~enable" t))
(setq *reconfigure-server*
      (def-dynamic-reconfigure-server
        ;;; ((name type level description (default) (min) (max) (edit_method)) ... )
        (("volume"  double_t  0  "tweet speak volume" 1.0  0.0 1.0)
         ("speak_enable"  bool_t  0  "tweet speak enable" t)
         ("enable"  bool_t  0  "tweet enable" t))
        ;; use lamda-closure to avoid memory error
        '(lambda-closure nil 0 0 (cfg level)
            (let ((prev-volume *volume*)
                  (prev-speak-enable *speak-enable*)
                  (prev-enable *enable*))
              (setq *volume* (cdr (assoc "volume" cfg :test #'equal)))
              (setq *speak-enable* (cdr (assoc "speak_enable" cfg :test #'equal)))
              (setq *enable* (cdr (assoc "enable" cfg :test #'equal)))
              (if (null (equal *volume* prev-volume))
                (ros::ros-warn "Volume changed to: ~A" *volume*))
              (if (null (equal *enable* prev-enable))
                (ros::ros-warn "Enable changed to: ~A" *enable*))
              (if (null (equal *speak-enable* prev-speak-enable))
                (ros::ros-warn "Speak enable changed to: ~A" *speak-enable*)))
            cfg)))

(ros::advertise "/tweet" std_msgs::String 1)
(ros::rate 0.1)
(do-until-key
  (if *enable*
    (progn
      (setq *user-name* (ros::get-param "/active_user/launch_user_name")
            *elapsed-time* (ros::get-param "/active_user/elapsed_time"))
      ;; tweet depend on up time
      (let ((st (ros::get-param "/active_user/start_time")))
        (when st
          (let ((waking-time (- (send (ros::time-now) :to-sec) st)))
            (ros::ros-debug "~A waking ~A sec (~A)" *robot-name* waking-time *waking-target-second*)
            (when (> waking-time *waking-target-second*)
              (incf *waking-target-second* *waking-tweet-second*)
              ;; tweet source of robot-interface
              (unless *src-lines*
                (let* ((dirname (ros::rospack-find "pr2eus"))
                       (fname   (format nil "~A/robot-interface.l" dirname))
                       str)
                  (with-open-file (f fname)
                    (while (setq str (read-line f nil nil))
                    (push str *src-lines*)))
                  (setq *src-lines* (nreverse *src-lines*))
                  ))

              (let* ((len (length *src-lines*))
                     (start-n (floor (random (float len) *random-state*)))
                     (spos 0) (str-len 0) lines)
                (push (format nil "I am running ~A min." (round (/ waking-time 60.0)))
                      lines)
                (incf str-len (length (car lines)))
                (while (< (+ start-n spos) len)
                  (let ((str (elt *src-lines* (+ start-n spos))))
                    (incf str-len (length str))
                    (if (> str-len 140) (return))
                    (push str lines))
                  (incf spos))
                (let* ((ln (apply #'+ (length lines)
                                  (mapcar #'(lambda (x) (length x)) lines)))
                       (dt (make-string (1- ln)))
                       (pos 0))
                  (dolist (s (nreverse lines))
                    (replace dt s :start1 pos)
                    (incf pos (length s))
                    (if (< pos (- ln 2)) (setf (elt dt pos) 10))
                    (incf pos))
                  (tweet-string dt :warning-time 1 :with-image t
                                :speak *speak-enable*
                                :volume *volume*)
                  ))))))))
  (ros::spin-once)
  (ros::sleep)
  )
