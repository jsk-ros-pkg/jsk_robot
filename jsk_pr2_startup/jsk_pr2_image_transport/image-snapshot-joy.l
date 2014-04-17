#!/usr/bin/env roseus
(ros::load-ros-manifest "jsk_pr2_startup")
(ros::load-ros-manifest "jsk_teleop_joy")

(ros::roseus "image-snapshot-joy")

(defun callback-joy (joy)
  (let* ((buttons (send joy :buttons)))
    
    (print "hoge")
  )
)
;;(ros::subscribe "/joy" sensor_msgs
(ros::subscribe "/interactive_control_joy" sensor_msgs
                #'callback-joy)


(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (ros::sleep)
 )
