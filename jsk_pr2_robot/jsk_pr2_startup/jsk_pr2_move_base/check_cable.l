#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sound_play")
(ros::roseus-add-msgs "pr2_msgs")
(load "package://pr2eus/speak.l")

(setq *ac* nil *ac-tm* (ros::time 0)) ;; ac_present > 0
(setq *net* nil) ;; netwoork is connected

(setq *ac-warning-time* (ros::time 0)) ;;last network warning
(setq *ac-warning-sec*  5)     ;;warn at fixed intervals

(setq *net-warning-time* (ros::time 0)) ;;last network warning
(setq *net-warning-sec*  (* 60 10))     ;;warn at fixed intervals

(defun move-base-command-cb (msg)
  (cond
   (*ac*
    (when (< *ac-warning-sec* (send (ros::time- (ros::time-now) *ac-warning-time*) :to-sec))
      (setq *ac-warning-time* (ros::time-now))
      (speak-jp (format nil "でんげんをぬいてください"))
      )
    )
   (*net*
    (when (< *net-warning-sec* (send (ros::time- (ros::time-now) *net-warning-time*) :to-sec))
      (setq *net-warning-time* (ros::time-now))
      (speak-jp (format nil "らんけーぶるがささっています"))
      )
    (ros::publish "/base_controller/command" msg)
    )
   (t
    (ros::publish "/base_controller/command" msg)
    )
   )
  )

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

(ros::roseus "check_cable")

(ros::subscribe "/power_state" pr2_msgs::PowerState #'check-power-state-cb)
(ros::subscribe "/network/connected" std_msgs::Bool #'check-network-connected-cb)

(ros::advertise "/base_controller/command" geometry_msgs::Twist)
(ros::subscribe "/base_controller/command_unchecked" geometry_msgs::Twist #'move-base-command-cb)

(ros::advertise "/robotsound_jp" sound_play::SoundRequest 5)
(ros::spin)
