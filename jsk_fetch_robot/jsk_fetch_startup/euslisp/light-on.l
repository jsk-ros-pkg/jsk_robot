#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(ros::roseus "light_on")
(fetch-init)
(room-light-on :control-switchbot :api)
(sys::exit 0)
