#!/usr/bin/env roseus

(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(ros::roseus "fetch_undock")
(undock)
(sys::exit 0)
