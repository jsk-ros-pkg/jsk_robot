#!/usr/bin/env roseus


(defclass tuck-arm-teleop-class
    :slots ((dualsense-deadman (ros::get-param
                                       "/teleop_twist_joy_dualsense/enable_button" 4))
            (dualsense-tuck (ros::get-param
                                    "/teleop_twist_joy_dualsense/tuck_button" 6))
            (R1-deadman (ros::get-param
                                "/teleop_twist_joy_dualsense/enable_button" nil))))
(defmethod tuck-arm-teleop-class
  (:init ()
   (ros::roseus "kinova-rest-teleop")
   (ros::roseus-add-msgs "sensor_msgs")
   (ros::subscribe "/joy_dualsense" sensor_msgs::Joy #'send self :dualsense-cb)
   (ros::subscribe "/joy_R1" sensor_msgs::Joy #'send self :r1-cb))
  (:dualsense-cb (msg)
   (when (and (> (elt (send msg :buttons) 4) 0)
              (< (elt (send msg :axes) 6) 0))
     (send self :tuck-arm)))
  (:tuck-arm ()
   
   )
