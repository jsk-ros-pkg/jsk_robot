#!/usr/bin/env roseus

(ros::roseus "speak_volume_zero")
(ros::set-dynamic-reconfigure-param "/audible_warning" "volume" :double 0.0)
(ros::set-dynamic-reconfigure-param "/tweet_client_tablet" "volume" :double 0.0)
(ros::set-dynamic-reconfigure-param "/tweet_client_uptime" "volume" :double 0.0)
(ros::set-dynamic-reconfigure-param "/tweet_client_warning" "volume" :double 0.0)
(ros::set-dynamic-reconfigure-param "/tweet_client_worktime" "volume" :double 0.0)
(exit)
