#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(load "package://jsk_robot_startup/lifelog/look-at-human.l")

;; Main process
(ros::roseus "pr2_look_at_human")
(pr2-init)
(main *pr2* (send *pr2* :kinect_head-rgb))
