#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(unless (boundp '*spot*) (spot) (send *spot* :reset-pose))
(unless (ros::ok) (ros::roseus "spot_eus_interface"))
(unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))
(send *ri* :claim)

(setq ip "unknown")
(setq host (unix::gethostbyname "spotcore.jsk.imi.i.u-tokyo.ac.jp"))
(if (listp host)
    (setq ip (format nil "~A . ~A . ~A . ~A" (elt (car host) 0) (elt (car host) 1) (elt (car host) 2) (elt (car host) 3))))

(setq info-txt (format nil "Hello, my name is ~A, my internet address is ~A, my owner is ~A and my dock id is ~A"
		       (ros::get-param "/robot/name")
		       ip
		       (unix::getenv "USER")
		       (round (ros::get-param "/spot/spot_ros/dock_id"))))
(ros::ros-info info-txt)
(send *ri* :speak info-txt :wait t)


(ros::exit)
