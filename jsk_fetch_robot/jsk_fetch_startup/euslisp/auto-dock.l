#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(ros::roseus "fetch_go_dock")
(fetch-init)
(send *ri* :clear-costmap)
(auto-dock)
(exit)
