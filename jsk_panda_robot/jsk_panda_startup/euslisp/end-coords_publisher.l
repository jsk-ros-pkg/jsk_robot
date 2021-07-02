#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::roseus "ee-poses-publisher")

(defmethod dual_panda-robot-interface
  (:dummy-controller (&rest args)))
(setq *ri* (instance dual_panda-robot-interface :init
                     :type :dummy-controller))

(setq *robot* (dual_panda))

(ros::advertise "/dual_panda/end_coords/rarm" geometry_msgs::PoseStamped 1)
(ros::advertise "/dual_panda/end_coords/larm" geometry_msgs::PoseStamped 1)

(defvar *joints* nil)

(ros::rate (ros::get-param "/dual_panda/end_coords_pub_hz"))



(while (ros::ok)
  (let ((pose-stamped-rarm (instance geometry_msgs::PoseStamped :init) (pose-stamped-larm (instance geometry_msgs::PoseStamped :init))))
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (setq pose-stamped-rarm (ros::coords->tf-pose-stamped (send *robot* :rarm :end-coords) "dual_arm_base"))
    (setq pose-stamped-larm (ros::coords->tf-pose-stamped (send *robot* :larm :end-coords) "dual_arm_base"))
    (ros::publish "/dual_panda/end_coords/rarm" pose-stamped-rarm)
    (ros::publish "/dual_panda/end_coords/larm" pose-stamped-larm)
    (ros::spin-once)
    (ros::sleep)))
