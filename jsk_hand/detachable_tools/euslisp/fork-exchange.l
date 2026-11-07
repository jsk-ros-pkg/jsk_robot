;; ROS 2 wrapper for the shared nextage_fork_server (metres/radians on server).
;; Does not start a server or move the robot when loaded.
(defun nextage-fork-call (operation &key (namespace "/fork_exchange"))
  (ros::load-ros-package "std_srvs")
  (let* ((srv (format nil "~A/~A" namespace operation))
         (request (instance (symbol-value (read-from-string "std_srvs::TriggerRequest")) :init))
         response)
    (unless (ros::wait-for-service srv 3)
      (error "Fork API server is unavailable: ~A" srv))
    (setq response (ros::service-call srv request))
    (unless (and response (send response :success))
      (error "Fork operation failed: ~A" (and response (send response :message))))
    ;; Return the JSON report. It includes dry_run and executed flags.
    (send response :message)))

(defun nextage-fork-plan-detach (&key (namespace "/fork_exchange"))
  (nextage-fork-call "plan_detach" :namespace namespace))
(defun nextage-fork-plan-attach (&key (namespace "/fork_exchange"))
  (nextage-fork-call "plan_attach" :namespace namespace))
(defun nextage-fork-detach (&key (namespace "/fork_exchange"))
  (nextage-fork-call "detach" :namespace namespace))
(defun nextage-fork-attach (&key (namespace "/fork_exchange"))
  (nextage-fork-call "attach" :namespace namespace))
