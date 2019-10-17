#!/usr/bin/env roseus

(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(ros::roseus "go_to_kitchen")
(go-to-kitchen :tweet t :n-dock-trial 5)
(exit)
