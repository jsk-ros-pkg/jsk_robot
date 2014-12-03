#!/usr/bin/env roseus

(ros::roseus "twitter_client_warning")

(load "package://jsk_robot_startup/lifelog/tweet_client.l")
(ros::load-ros-manifest "diagnostic_msgs")

(defun diagnostics-cb (msg)
  (let ((diagnostics (make-hash-table :test #'equal))
	(tm (ros::time-now))
	status id)
    (format t "~0,3f diagnostics_msgs~%" (send tm :to-sec))
    (dolist (status (send msg :status))
      ;; diagnostic_msgs::DiagnosticStatus::*WARN*
      (when (>= (send status :level) diagnostic_msgs::DiagnosticStatus::*WARN*) ;; diagnostic_msgs::DiagnosticStatus::*ERROR*)
	(cond ((substringp "/Motors" (send status :name))
	       t) ;; skip motors
	      ((substringp "/Other/Accelerometer" (send status :name)) t)
	      ((substringp "/Other/Pressure" (send status :name)) t)
	      ((and (string= "/Computers/Network/Wifi Status (ddwrt)" (send status :name))
		    (string= "Updates Stale" (send status :message))) t)
	      ((and (string= "/Computers/Network" (send status :name))
		    (string= "Error" (send status :message))) t)
	      ((position #\/ (send status :name) :count 2) ;; check depth of name
	       (setq key (subseq (send status :name) 0 (position #\/ (send status :name) :count 2)))
	       (when (> (length (send status :name)) (length (gethash key diagnostics)))
		 (setf (gethash key diagnostics) (cons (send status :name) (send status :message)))
		 ) ;; when
	       ))
	)) ;; when / dolist
    (maphash #'(lambda (k v) (format t "Warnings ~A ~A~%" (length status) v) (push v status)) diagnostics)
    (when status
      (setq id (random (length status)))
      (when (= (mod (round (send tm :sec)) 1000) 0)
	(tweet-string (format nil "Warning!! ~A is ~A at ~0,3f" (car (elt status id)) (cdr (elt status id)) (send tm :to-sec)))
	)) ;; when
    )) ;; let

(ros::subscribe "/diagnostics_agg" diagnostic_msgs::DiagnosticArray #'diagnostics-cb)
(ros::rate (/ 1.0 3.0))
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )
