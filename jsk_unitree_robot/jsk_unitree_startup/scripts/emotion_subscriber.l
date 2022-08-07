#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(ros::roseus "emotion-lister")
(load "package://unitreeeus/unitree-interface.l")

(require "package://jsk_unitree_startup/scripts/motions/happy.l")
(require "package://jsk_unitree_startup/scripts/motions/joy.l")
(require "package://jsk_unitree_startup/scripts/motions/affirmation.l")
(require "package://jsk_unitree_startup/scripts/motions/negation.l")
(require "package://jsk_unitree_startup/scripts/motions/love.l")
(require "package://jsk_unitree_startup/scripts/motions/scared.l")
(require "package://jsk_unitree_startup/scripts/motions/curious.l")
(require "package://jsk_unitree_startup/scripts/motions/astonished.l")


(defun emotion-cb(msg)
  (let ((emotion (send msg :data)))
    (ros::ros-info "Callback chatting-cb called with ~A" emotion)
    (cond ((string= emotion "happy")
	   (happy-main))
	  ((string=  emotion "joy")
	   (joy-main))
	  ((string= emotion "affirmation")
	   (affirmation-main))
	  ((string= emotion "negation")
	   (negation-main))
	  ((string= emotion "love")
	   (love-main))
	  ((string= emotion "scared")
	   (scared-main))
	  ((string= emotion "curious")
	   (curious-main))
	  ((string= emotion "astonished")
	   (astonished-main))
	  (t
	   (ros::ros-warn "called unknown emotion ~A" emotion)))
    emotion))

(defun main()
  (go1-init)
  (ros::subscribe "/emotion" std_msgs::String #'emotion-cb)
  (ros::rate 10)
  (while (ros::ok)
    (ros::spin-once)))

(main)
