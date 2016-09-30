#!/usr/bin/env roseus
;; initialpose_publisher.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "initialpose_publisher")

(ros::ros-warn "[DEPLICATED] initialpose_publisher.l is moved to jsk_robot_startup/util")

(load "package://jsk_robot_startup/util/initialpose_publisher.l")
