#!/usr/bin/env roseus

(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(ros::roseus "fetch_dock")
(simple-dock)
(sys::exit 0)
