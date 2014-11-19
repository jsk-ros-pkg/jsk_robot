#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sound_play")
(ros::roseus-add-msgs "pr2_msgs")
(ros::roseus-add-msgs "actionlib_msgs")
(ros::roseus-add-srvs "std_srvs")

(setq *ac* nil *ac-tm* (ros::time 0)) ;; ac_present > 0
(setq *net* nil) ;; netwoork is connected
(setq *motor* t) ;; if motors working

(defun play_sound (sound)
  (let ((msg (instance sound_play::SoundRequest :init)))
    (cond
     ((numberp sound)
      (send msg :sound sound))
     (t
      (send msg :sound sound_play::SoundRequest::*play_file*)
      (send msg :arg sound)))
    (send msg :command sound_play::SoundRequest::*play_once*)
    (ros::publish "/robotsound" msg)))

(defun stop_robot ()
  (ros::service-call "/pr2_etherCAT/halt_motors" (instance std_srvs::EmptyRequest :init)))

(defun motors-halted-cb (msg)
  (setq *motor* (not (send msg :data))))

(defun move-base-status-cb (msg)
  (ros::ros-info "base status motor:~A ac:~A, net:~A, goal:~A" *motor* *ac* *net* (send-all (send msg :status_list) :status))
  (when (and *motor*
	     (or *ac* *net*)
	     (some #'(lambda (x)
		       (eq (send x :status)
			   actionlib_msgs::GoalStatus::*active*))
		   (send msg :status_list)))
    (play_sound sound_play::SoundRequest::*needs_unplugging_badly*)
    (stop_robot)))

(defun check-power-state-cb (msg)
  (let ((p (send msg :relative_capacity))
	(a (send msg :ac_present))
	(tm (ros::time-now)))
    (ros::ros-info "power-state pc:~A cap:~A" p a)
    (setq *ac* (> a 0))
    (if (or *ac*
	    (> 180.0 (send (ros::time- tm *ac-tm*) :to-sec)))
	(return-from check-power-state-cb nil))
    (setq *ac-tm* tm) ;; check every 3min
    ))

(defun check-network-connected-cb (msg)
  (setq *net* (send msg :data)))

(ros::roseus "cable_warning")
(ros::wait-for-service "/pr2_etherCAT/halt_motors")
(ros::subscribe "/power_state" pr2_msgs::PowerState #'check-power-state-cb)
(ros::subscribe "/network/connected" std_msgs::Bool #'check-network-connected-cb)
(ros::subscribe "/pr2_etherCAT/motors_halted" std_msgs::Bool #'motors-halted-cb)
(ros::subscribe "/move_base/status" actionlib_msgs::GoalStatusArray #'move-base-status-cb)
(ros::advertise "/robotsound" sound_play::SoundRequest 10)
(ros::spin)
