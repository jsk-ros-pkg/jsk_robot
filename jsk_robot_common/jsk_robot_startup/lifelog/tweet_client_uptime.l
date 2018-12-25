#!/usr/bin/env roseus

(ros::roseus "twitter_client_uptime")

(require :tweet_client "package://jsk_robot_startup/lifelog/tweet_client.l")

(setq *src-lines* nil)
(setq *start-time* nil)
(setq *random-state* (coerce (unix::gettimeofday) integer-vector))

(defvar *uptime-tweet-interval* (ros::get-param "~uptime_tweet_interval" 3600.0)) ;; [secs]

(defun tweet-uptime (&optional event)
  ;; tweet source of robot-interface
  (unless *start-time*
    (setq *start-time* (ros::get-param "/active_user/start_time" nil)))
  (unless *start-time* (return-from tweet-uptime nil))
  (unless *src-lines*
    (let* ((dirname (ros::rospack-find "pr2eus"))
           (fname   (format nil "~A/robot-interface.l" dirname))
           str)
      (with-open-file (f fname)
        (while (setq str (read-line f nil nil))
          (push str *src-lines*)))
      (setq *src-lines* (nreverse *src-lines*))))
  ;;
  (let* ((len (length *src-lines*))
         (uptime (- (send (ros::time-now) :to-sec) *start-time*))
         (start-n (floor (random (float len) *random-state*)))
         (spos 0) (str-len 0) lines)
    (push (format nil "I am running ~A min." (round (/ uptime 60.0)))
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
      )))

(defun main ()
  (ros::create-timer *uptime-tweet-interval* #'tweet-uptime)
  (ros::spin))

(main)
