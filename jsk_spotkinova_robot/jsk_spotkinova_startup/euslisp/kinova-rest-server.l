#!/usr/bin/env roseus

(defclass kinova-arm-rest-server
  :slots ()
  :documentation "kinova-arm-rest-server is a service for making a rest for kinova on Spot
                  Examples:
                    $ rosservice call /arm_gen3/rest \"{}\" ")
(defmethod kinova-arm-rest-server
  (:init ()
   "Create service for spotkinova-rest-arm"
   (ros::roseus "kinova_rest")
   (ros::roseus-add-srvs "std_srvs")
   (load "package://spotkinovaeus/spotkinova-interface.l")
   (let* ((msg (one-shot-subscribe "/arm_gen3/joint_states" sensor_msgs::JointState
                                    :timeout 5000)))
     (when msg
       (spotkinova-init)
       (ros::ros-info "kinova arm rest server started")
       (ros::advertise-service "/arm_gen3/rest" std_srvs::Trigger
                               #'send self :rest-arm))))
  (:rest-arm (req)
    (let ((r (send req :response)))
      (send r :success
            (progn
              (send *ri* :state :potentio-vector)
              (send *ri* :angle-vector (send *spotkinova* :kinova-rest-pose) :fast)
              (send *ri* :wait-interpolation)))
      r)))

(setq a (instance kinova-arm-rest-server :init))
(ros::rate 1)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
