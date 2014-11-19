#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sound_play")
(ros::roseus-add-msgs "pr2_msgs")
(ros::roseus-add-msgs "actionlib_msgs")
(ros::roseus-add-srvs "std_srvs")
(load "package://pr2eus/speak.l")

(setq *ac* nil *ac-tm* (ros::time 0)) ;; ac_present > 0
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
    (speak-jp (format nil "でんちのこり~Aパーセントです" p))
    (cond 
	  ((< p 10)
	   (ros::ros-info "need to back to M78 nebula")
	   (play_sound sound_play::SoundRequest::*needs_plugging*)
	   )
	  ((< p 20)
	   (ros::ros-info "less than 20 perrcept")
	   ;;(play_sound sound_play::needs_unplugging)
	   (play_sound (ros::resolve-ros-path "package://piano/notes/E.wav"))
	   (play_sound (ros::resolve-ros-path "package://piano/notes/G.wav"))
	   (play_sound (ros::resolve-ros-path "package://piano/notes/B.wav"))
	   )
	  (t
	   ))))

(ros::roseus "battery_warning")
(ros::subscribe "/power_state" pr2_msgs::PowerState #'check-power-state-cb)
(ros::advertise "/robotsound" sound_play::SoundRequest 10)
(ros::advertise "/robotsound_jp" sound_play::SoundRequest 5)
(ros::spin)
