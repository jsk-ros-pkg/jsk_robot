#!/usr/bin/env roseus

(ros::roseus "twitter_client_uptime")

(load "package://jsk_robot_startup/lifelog/tweet_client.l")

(setq *src-lines* nil)
(setq *random-state* (coerce (unix::gettimeofday) integer-vector))

(defvar *robot-name* "robot")
(when (ros::has-param "/active_user/robot_name")
  (setq *robot-name* (ros::get-param "/active_user/robot_name"))
  )

(ros::rate 0.1)
(do-until-key
  (setq *user-name* (ros::get-param "/active_user/launch_user_name")
        *elapsed-time* (ros::get-param "/active_user/elapsed_time"))
  ;; tweet depend on up time
  (let ((st (ros::get-param "/active_user/start_time")))
    (when st
      (let ((waking-time (- (send (ros::time-now) :to-sec) st)))
        (ros::ros-info "~A waking ~A sec (~A)" *robot-name* waking-time *waking-target-second*)
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
              (tweet-string dt)
              ))
          ))))
  (ros::sleep)
  )
