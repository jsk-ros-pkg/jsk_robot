#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(ros::roseus "light_off")
(fetch-init)
(room-light-off :control-switchbot :api)
(sys::exit 0)
