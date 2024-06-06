#!/usr/bin/env roseus

(defclass kinova-arm-rest-teleop
    :slots (dualsense-deadman
            dualsense-rest
            rest-req
            rest-res))
(defmethod kinova-arm-rest-teleop
  (:init ()
   "create class for kinova-arm-teleop"
   (ros::roseus "kinova-rest-teleop")
   (ros::roseus-add-msgs "sensor_msgs")
   (ros::roseus-add-srvs "std_srvs")
   (ros::subscribe "/joy_dualsense" sensor_msgs::Joy #'send self :dualsense-cb)
   (setq dualsense-deadman (ros::get-param "/teleop_twist_joy_dualsense/enable_button" 4))
   (setq dualsense-rest (ros::get-param "/teleop_twist_joy_dualsense/kinova_rest_button" 6))
   (ros::wait-for-service "arm_gen3/rest"))
  (:dualsense-cb (msg)
   (when (and (> (elt (send msg :buttons) (floor dualsense-deadman)) 0)
              (< (elt (send msg :axes) (floor dualsense-rest)) 0))
     (send self :arm-rest)))
  (:arm-rest ()
   (setq rest-req (instance std_srvs::TriggerRequest :init))
   (setq rest-res (ros::service-call "arm_gen3/rest" rest-req))
   (send rest-res :success)
   (ros::ros-info "kinova rest"))
   )

(setq m (instance kinova-arm-rest-teleop :init))
(ros::rate 1)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
