#!/usr/bin/env roseus

(ros::roseus "speak_volume_zero")
(ros::set-dynamic-reconfigure-param "/audible_warning" "volume" :double 0.0)
(exit)
