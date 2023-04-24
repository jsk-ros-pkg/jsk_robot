#!/usr/bin/env roseus

(ros::roseus "twitter_client_warning")

(load "package://roseus/euslisp/dynamic-reconfigure-server.l")
(load "package://jsk_robot_startup/lifelog/tweet_client.l")
(ros::load-ros-manifest "diagnostic_msgs")

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

(defun diagnostics-cb (msg)
  (if *enable*
    (let ((diagnostics (make-hash-table :test #'equal))
          (tm (ros::time-now))
          status id)
      (ros::ros-debug (format nil "~0,3f diagnostics_msgs~%" (send tm :to-sec)))
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
                ((substringp "/Peripherals/PS3 Controller" (send status :name)) t) ;; fetch joystick warning
                ((position #\/ (send status :name) :count 2) ;; check depth of name
                 (setq key (subseq (send status :name) 0 (position #\/ (send status :name) :count 2)))
                 (when (> (length (send status :name)) (length (gethash key diagnostics)))
                   (setf (gethash key diagnostics) (cons (send status :name) (send status :message)))
                   ) ;; when
                 ))
          )) ;; when / dolist
      (maphash #'(lambda (k v) (ros::ros-debug (format nil "Warnings ~A ~A~%" (length status) v)) (push v status)) diagnostics)
      (when status
        (setq id (random (length status)))
        (when (= (mod (round (send tm :sec)) 1000) 0)
          (tweet-string (format nil "Warning!! ~A is ~A at ~0,3f" (car (elt status id)) (cdr (elt status id)) (send tm :to-sec))
                        :warning-time 1 :with-image t :volume *volume* :speak *speak-enable*)))
      )))

(ros::advertise "/tweet" std_msgs::String 1)
(ros::subscribe "/diagnostics_agg" diagnostic_msgs::DiagnosticArray #'diagnostics-cb)
(ros::rate (/ 1.0 3.0))
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )
