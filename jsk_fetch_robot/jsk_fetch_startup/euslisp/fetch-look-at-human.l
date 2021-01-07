#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(load "package://jsk_robot_startup/lifelog/look-at-human.l")

;; Main process
(ros::roseus "fetch_look_at_human")
(fetch-init)
(main *fetch* (send *fetch* :head-rgb))
