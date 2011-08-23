#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "diagnostic_msgs")
(load "package://pr2eus/pr2-interface.l")

(setq *last-motors-error-value* nil)
(setq *current-motors-error-value* nil)
(setq *need-angle-vector* nil)
(defun diagnostics-cb (msg)
  (let (kv)
    (dolist (status (send msg :status))
      (setq kv (find "Halt motors error count" (send status :values) :key #'(lambda (x) (send x :key)) :test #'string=))
      (when kv
	(setq *current-motors-error-value* (read-from-string (send kv :value)))
	(if (null *last-motors-error-value*)
	    (setq *last-motors-error-value* *current-motors-error-value*)))
      )))

(defun motor-cb (msg)
  (format t "/pr2_etherCAT/motors_halted -> ~A (count ~A->~A)~%" (send msg :data) *last-motors-error-value* *current-motors-error-value*)
  (cond ((send msg :data)
	 (warning-message 2 "Halt motors error count ~A -> ~A~%"
			  *last-motors-error-value*
			  *current-motors-error-value*)
	 (when (and *last-motors-error-value*
		    (< *last-motors-error-value*  *current-motors-error-value*)
		    )
	   (setq  *last-motors-error-value* *current-motors-error-value*)
	   (setq *need-angle-vector* t)))))

(ros::roseus "reset_motor")
(ros::subscribe "/pr2_etherCAT/motors_halted" std_msgs::Bool #'motor-cb)
(ros::subscribe "/diagnostics" diagnostic_msgs::DiagnosticArray #'diagnostics-cb)

(setq *ri* (instance pr2-interface :init))
(setq *av-list* (list (send *ri* :state :potentio-vector)))
(ros::rate 10)
(do-until-key
 (setq *av-list* (nconc *av-list* (list (send *ri* :state :potentio-vector))))
 (if (>= (length *av-list*) 100) (setq *av-list* (cdr *av-list*)))
 (ros::spin-once)
 (when *need-angle-vector*
   (ros::ros-warn "reseted motors")
   (call-empty-service "/pr2_etherCAT/reset_motors")
   (send *ri* :head-angle-vector (car *av-list*) 100)
   (setq *need-angle-vector* nil)
   (unix::sleep 2))
 (ros::sleep))
