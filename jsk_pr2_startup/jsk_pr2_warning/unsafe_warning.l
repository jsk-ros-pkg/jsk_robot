#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sound_play")
(ros::roseus-add-msgs "sensor_msgs")

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

(defun joy-cb (msg)
  (when (and
	 (eq 1 (elt (send msg :buttons) 9))
	 (eq 1 (elt (send msg :buttons) 10)))
    (play_sound 2) ;; kan kan
    (unix::sleep 2)
    ))

(ros::roseus "unsafe_warning")
(ros::subscribe "/joy" sensor_msgs::joy #'joy-cb)
(ros::advertise "/robotsound" sound_play::SoundRequest 10)
(ros::spin)
