#!/usr/bin/env roseus

(ros::roseus "speak_volume_lower")
(ros::set-dynamic-reconfigure-param "/audible_warning" "volume" :double 0.2)
(exit)
