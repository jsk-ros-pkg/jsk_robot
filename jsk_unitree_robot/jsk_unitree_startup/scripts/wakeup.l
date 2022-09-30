#!/usr/bin/env roseus

;; load simple model for fast-boot
(require :go1 "package://unitreeeus/go1-simple.l")

;; copied from https://github.com/jsk-ros-pkg/jsk_demos/blob/0a82540da013b8e21307354bb1c6a552185e36a2/jsk_spot_watch_dog/scripts/watch-dog.l
(load "package://unitreeeus/unitree-interface.l")

(go1-init)
(ros::ros-info "go1-initialized")
(send *ri* :play-sound
      (pathname (ros::get-param "~bark_sound" "/opt/jsk/User/src/jsk_robot/jsk_unitree_robot/jsk_unitree_startup/autostart/bark.wav")))
(ros::ros-info "send :play-sound, wait for 2 sec..")
(unix::sleep 2)
(ros::ros-info "exitting...")
(sys::exit 0)
