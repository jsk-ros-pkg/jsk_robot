#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")

(defun motor-cb (msg)
  (print (list 'halt (send msg :data)))
  (cond ((send msg :data)
	 (call-empty-service "/pr2_etherCAT/reset_motors")
	 (ros::ros-warn "reseted motors")
	 (unix::sleep 2)))
  )


(ros::roseus "reset_motor")
(ros::subscribe "/pr2_etherCAT/motors_halted" std_msgs::Bool #'motor-cb)
(ros::spin)
