#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(defun main ()
  (fetch-init)
  (send *ri* :clear-costmap)
  (go-to-kitchen :tweet t :n-dock-trial 3 :n-kitchen-trial 3))

(ros::roseus "go_to_kitchen")
(if (main) (unix::exit 0) (unix::exit 1))
