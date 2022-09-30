#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sound_play")
(ros::roseus-add-msgs "pr2_msgs")
(load "package://pr2eus/speak.l")

(defparameter *check-network-p* t)
(defparameter *check-power-p* t)
(defparameter *ac* t)
(defparameter *net* t)
(defparameter *ac-tm* (ros::time 0))

(setq *ac-warning-time* (ros::time 0)) ;;last network warning
(setq *ac-warning-sec*  5)     ;;warn at fixed intervals

(setq *net-warning-time* (ros::time 0)) ;;last network warning
(setq *net-warning-sec*  (* 60 10))     ;;warn at fixed intervals

(defun move-base-command-cb (msg)
  (cond
   ((and (= (send msg :linear :x) 0)
         (= (send msg :linear :y) 0)
         (= (send msg :linear :z) 0)
         (= (send msg :angular :x) 0)
         (= (send msg :angular :y) 0)
         (= (send msg :angular :z) 0))
    (ros::publish "/base_controller/command" msg))
   ((and *check-power-p* *ac*)
    (when (< *ac-warning-sec* (send (ros::time- (ros::time-now) *ac-warning-time*) :to-sec))
      (setq *ac-warning-time* (ros::time-now))
      (speak-jp (format nil "電源ケーブルを抜いてください。") :wait nil)
      )
    )
   ((and *check-network-p* *net*)
    (when (< *net-warning-sec* (send (ros::time- (ros::time-now) *net-warning-time*) :to-sec))
      (setq *net-warning-time* (ros::time-now))
      (speak-jp (format nil "LANケーブルが刺さっています。") :wait nil)
      )
    (ros::publish "/base_controller/command" msg)
    )
   (t
    (ros::publish "/base_controller/command" msg)
    )))

(defun check-power-state-cb (msg)
  (let ((p (send msg :relative_capacity))
	(a (send msg :ac_present))
	(tm (ros::time-now)))
    (ros::ros-debug "power-state pc:~A cap:~A" p a)
    (when (and *ac* (<= a 0))
      (speak-jp (format nil "PR2、移動可能です。") :wait nil))
    (setq *ac* (> a 0))
    (if (or *ac*
	    (> 180.0 (send (ros::time- tm *ac-tm*) :to-sec)))
	(return-from check-power-state-cb nil))
    (setq *ac-tm* tm) ;; check every 3min
    ))

(defun check-network-connected-cb (msg)
  (setq *net* (send msg :data)))

(ros::roseus "cable_warning")

(setq *check-power-p* (ros::get-param "~check_power" t))
(setq *check-network-p* (ros::get-param "~check_network" t))

(ros::ros-info "check network: ~A power: ~A" *check-network-p* *check-power-p*)

(if *check-power-p*
    (ros::subscribe "/power_state" pr2_msgs::PowerState #'check-power-state-cb)
    (setq *ac* nil))
(if *check-network-p*
    (ros::subscribe "/network/connected" std_msgs::Bool #'check-network-connected-cb)
    (setq *net* nil))

(ros::advertise "/base_controller/command" geometry_msgs::Twist)
(ros::subscribe "/base_controller/command_unchecked" geometry_msgs::Twist #'move-base-command-cb)

(ros::advertise "/robotsound_jp" sound_play::SoundRequest 5)
(ros::spin)
(exit)
