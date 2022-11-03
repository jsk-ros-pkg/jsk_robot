#!/usr/bin/env roseus

(ros::roseus "speak_volume_reset")
(ros::set-dynamic-reconfigure-param "/audible_warning" "volume" :double 1.0)
(exit)
