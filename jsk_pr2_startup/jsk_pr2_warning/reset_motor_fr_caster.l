#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "diagnostic_msgs")
(load "package://pr2eus/pr2-interface.l")

(setq *target-error-list* (list) )
(defun diagnostics-cb (msg)
  (let (kv)
    (setq include-motors nil)
    (dolist ( status (send msg :status))
    (when (substringp "/Motors/EtherCAT Device" (send status :name))
      (setq include-motors t)
      (setq keyval (find "Mode" (send status :values) :key #'(lambda (x) (send x :key)) :test #'string=))
        (if keyval
            (progn
              (cond ((or (substringp "fr_caster_l_wheel_motor" (send status :name))
                         (substringp "fr_caster_r_wheel_motor" (send status :name))
                         (substringp "fr_caster_rotation_motor" (send status :name)))
                     (unless (string= "SAFETY_LOCKOUT" (send keyval :value))
                       (return-from diagnostics-cb nil)))
                    (t ;; other motors
                     (unless (string= "OFF" (send keyval :value))
                       (return-from diagnostics-cb nil)))
                    );;cond
              ))))
    (if include-motors
        (progn
          (print "RESET")
          (call-empty-service "/pr2_etherCAT/reset_motors")
          )
      )))

(ros::roseus "reset_motor")
(ros::subscribe "/diagnostics_agg" diagnostic_msgs::DiagnosticArray #'diagnostics-cb)
(ros::spin)