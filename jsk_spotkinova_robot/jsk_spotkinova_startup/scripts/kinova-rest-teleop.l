#!/usr/bin/env roseus

(defclass kinova-arm-rest-teleop
    :slots ((dualsense-deadman (ros::get-param
                                       "/teleop_twist_joy_dualsense/enable_button" 4))
            (dualsense-tuck (ros::get-param
                                    "/teleop_twist_joy_dualsense/kinova_rest_button" 6))
            (R1-deadman (ros::get-param
                         "/teleop_twist_joy_dualsense/enable_button" nil))
            (req nil)
            (res nil)))
(defmethod kinova-arm-rest-teleop
  (:init ()
   (ros::roseus "kinova-rest-teleop")
   (ros::roseus-add-msgs "sensor_msgs")
   (ros::roseus-add-srvs "std_srvs")
   (ros::subscribe "/joy_dualsense" sensor_msgs::Joy #'send self :dualsense-cb)
   ;;(ros::subscribe "/joy_R1" sensor_msgs::Joy #'send self :r1-cb)
   (ros::wait-for-service "arm_gen3/rest"))
  (:dualsense-cb (msg)
   (when (and (> (elt (send msg :buttons) 4) 0)
              (< (elt (send msg :axes) 6) 0))
     (send self :arm-rest)))
  (:arm-rest ()
   (setq req (instance std_srvs::TriggerRequest :init))
   (setq res (ros::service-call "arm_gen3/rest" req))
   (send res :success)
   (ros::ros-info "kinova rest"))
   )

(setq m (instance kinova-arm-rest-teleop-class :init))
(ros::rate 1)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
