#!/usr/bin/env roseus

(require :spot-interface "package://spoteus/spot-interface.l")

(ros::console-bridge-set-log-level 3) ;; suppress TF warning

(spot-init)

(setq *pose1* (instance geometry_msgs::Pose :init))
(send *pose1* :position :x 0.75)
(send *pose1* :position :y 0.4)
(send *pose1* :orientation :w 1.0)

(setq *pose2* (instance geometry_msgs::Pose :init))
(send *pose2* :position :x 0.75)
(send *pose2* :position :y -0.4)
(send *pose2* :orientation :w 1.0)

(send *ri* :arm-cartesian-trajectory (list *pose1* *pose2*))
