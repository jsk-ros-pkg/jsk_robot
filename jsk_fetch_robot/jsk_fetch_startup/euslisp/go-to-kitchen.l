#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(ros::roseus "go_to_kitchen")
(fetch-init)
(send *ri* :clear-costmap)
(go-to-kitchen :tweet t :n-dock-trial 5)
(exit)
