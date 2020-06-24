#!/usr/bin/env roseus

(require :speak "package://pr2eus/speak.l")

(ros::roseus-add-msgs "sensor_msgs")

(defun joy-cb (msg)
  (when (and
	 (eq 1 (elt (send msg :buttons) 9))
	 (eq 1 (elt (send msg :buttons) 10)))
    (play-sound 2 :topic-name "/robotsound" :wait nil) ;; kan kan
    (unix::sleep 2)
    ))

(ros::roseus "unsafe_warning")
(ros::subscribe "/joy" sensor_msgs::joy #'joy-cb)
(ros::spin)
